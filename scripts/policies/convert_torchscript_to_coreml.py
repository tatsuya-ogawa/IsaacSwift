#!/usr/bin/env python3
import argparse
from pathlib import Path

import coremltools as ct
import torch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="TorchScript .pt file")
    parser.add_argument("--output", required=True, help="Output .mlpackage path")
    parser.add_argument("--input-name", default="observations")
    parser.add_argument("--output-name", default="actions")
    parser.add_argument("--input-shape", default="1,48")
    args = parser.parse_args()

    input_shape = tuple(int(dim) for dim in args.input_shape.split(","))
    model = torch.jit.load(args.input, map_location="cpu")
    model.eval()

    example_input = torch.zeros(*input_shape, dtype=torch.float32)
    with torch.no_grad():
        traced = torch.jit.trace(model, example_input)

    converted = ct.convert(
        traced,
        source="pytorch",
        convert_to="mlprogram",
        inputs=[ct.TensorType(name=args.input_name, shape=input_shape)],
        outputs=[ct.TensorType(name=args.output_name)],
        minimum_deployment_target=ct.target.iOS15,
    )

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    converted.save(str(output_path))


if __name__ == "__main__":
    main()
