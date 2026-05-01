"""Convert ANYmal `sea_net_jit2.pt` (LSTM actuator network) to CoreML.

The PyTorch module signature is:

    forward(x:        Tensor,                        # [batch=12, seq=1, 2]
            hc0:      Tuple[h: Tensor, c: Tensor]   # both [num_layers=2, batch=12, hidden=8]
           ) -> Tuple[torque: Tensor,               # [12]
                      (h_new, c_new)]

For CoreML stateful inference we expose:

  inputs:
    - x:  shape (12, 2)        — (pos_err, joint_vel) per joint
    - h0: shape (2, 12, 8)
    - c0: shape (2, 12, 8)
  outputs:
    - tau: shape (12,)         — torque per joint (in N·m, post out_scale=20)
    - h1: shape (2, 12, 8)
    - c1: shape (2, 12, 8)

Run from the repo root:

    source /tmp/isaac-actuator-venv/bin/activate
    python scripts/convert_anymal_actuator.py

Outputs `PolicyModels/anymal_actuator.mlpackage` (then compile to
`.mlmodelc` via `xcrun coremlcompiler compile`).
"""

import os
import numpy as np
import torch
import coremltools as ct

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO_ROOT, "isaac_policy_sources/Anymal_Policies/sea_net_jit2.pt")
OUT_PACKAGE = os.path.join(REPO_ROOT, "PolicyModels/anymal_actuator.mlpackage")

NUM_LAYERS = 2
BATCH = 12        # one per joint
HIDDEN = 8

inner = torch.jit.load(SRC, map_location="cpu").eval()


def _copy_lstm_weights(scripted_lstm, fresh_lstm: torch.nn.LSTM):
    sd_src = {k: v for k, v in scripted_lstm.state_dict().items()}
    sd_dst = fresh_lstm.state_dict()
    for k in sd_dst.keys():
        if k in sd_src:
            sd_dst[k] = sd_src[k]
    fresh_lstm.load_state_dict(sd_dst)


class Wrapper(torch.nn.Module):
    """Pure-Python LSTM stack (no scripted control flow) — easier to trace.

    Mirrors `LSTMsea.forward`:
        x:   [batch=12, 2]
        h0/c0: [num_layers=2, batch=12, hidden=8]

    We mul the input by `in_scale = [2.0, 0.25]` and the linear output by
    `out_scale = 20.0`, exactly as in the original module.
    """

    def __init__(self, jit_module: torch.nn.Module):
        super().__init__()
        self.in_scale  = torch.nn.Parameter(jit_module.in_scale.clone(),  requires_grad=False)
        self.out_scale = torch.nn.Parameter(jit_module.out_scale.clone(), requires_grad=False)

        self.lstm = torch.nn.LSTM(input_size=2, hidden_size=HIDDEN,
                                  num_layers=NUM_LAYERS, batch_first=False)
        _copy_lstm_weights(jit_module.lstm, self.lstm)

        self.linear = torch.nn.Linear(HIDDEN, 1)
        self.linear.weight.data.copy_(jit_module.linear.weight)
        self.linear.bias.data.copy_(jit_module.linear.bias)

    def forward(self, x_flat: torch.Tensor,
                h0: torch.Tensor,
                c0: torch.Tensor):
        # Reshape (12, 2) → (seq=1, batch=12, 2) and apply in_scale.
        x = x_flat.unsqueeze(0) * self.in_scale.view(1, 1, -1)
        out, (h1, c1) = self.lstm(x, (h0, c0))
        tau = self.linear(out.squeeze(0)).squeeze(-1) * self.out_scale.view(-1)
        return tau, h1, c1


wrapped = Wrapper(inner).eval()

example_x  = torch.zeros(BATCH, 2)
example_h0 = torch.zeros(NUM_LAYERS, BATCH, HIDDEN)
example_c0 = torch.zeros(NUM_LAYERS, BATCH, HIDDEN)

with torch.no_grad():
    tau_ref, h1_ref, c1_ref = wrapped(example_x, example_h0, example_c0)
print("Reference torque (zero input):", tau_ref.shape, tau_ref.tolist())

traced = torch.jit.trace(wrapped, (example_x, example_h0, example_c0), strict=False)

mlmodel = ct.convert(
    traced,
    inputs=[
        ct.TensorType(name="x",  shape=example_x.shape,  dtype=np.float32),
        ct.TensorType(name="h0", shape=example_h0.shape, dtype=np.float32),
        ct.TensorType(name="c0", shape=example_c0.shape, dtype=np.float32),
    ],
    outputs=[
        ct.TensorType(name="tau", dtype=np.float32),
        ct.TensorType(name="h1",  dtype=np.float32),
        ct.TensorType(name="c1",  dtype=np.float32),
    ],
    convert_to="mlprogram",
    minimum_deployment_target=ct.target.iOS17,
    compute_precision=ct.precision.FLOAT32,
)
mlmodel.short_description = "ANYmal-C ANYdrive ActuatorNetLSTM (sea_net_jit2.pt)"
mlmodel.author = "ETH Zurich / RSL — converted for IsaacSwift"
mlmodel.save(OUT_PACKAGE)
print(f"Wrote {OUT_PACKAGE}")
