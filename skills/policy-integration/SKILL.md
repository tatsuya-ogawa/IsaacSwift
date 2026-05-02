---
name: isaacswift-policy-integration
description: Use this skill when integrating, porting, tuning, or debugging Isaac Lab locomotion policies in IsaacSwift's Swift/Jolt simulator, including joint permutations, observation layout, PD gains, action scale, policy cadence, and final verification.
---

# Isaac Lab Policy → Swift / Jolt Integration Guide

This document describes how to integrate a quadruped, humanoid, or other reinforcement
learning policy that was trained inside **Isaac Sim / Isaac Lab** with the
local Swift + Jolt simulator in this repo. It is the post-mortem of the Spot
"falls over immediately" bug and a checklist for porting future models
(ANYmal-D, Unitree Go2, H1, etc.).

> Source of truth for code:
> - [`IsaacSwift/IsaacSwiftPhysics.h`](../../IsaacSwift/IsaacSwiftPhysics.h),
>   [`IsaacSwift/IsaacSwiftPhysics.mm`](../../IsaacSwift/IsaacSwiftPhysics.mm)
>   — Jolt-backed simulator (per-robot config tables).
> - [`IsaacSwift/PolicyModel.swift`](../../IsaacSwift/PolicyModel.swift)
>   — `IsaacPolicyRuntimeConfiguration`, `DemoPolicyActionProvider`.
> - [`IsaacSwift/PolicyPhysicsLoop.swift`](../../IsaacSwift/PolicyPhysicsLoop.swift)
>   — policy-tick orchestrator.
> - [`IsaacSwiftTests/IsaacSwiftPhysicsTests.swift`](../../IsaacSwiftTests/IsaacSwiftPhysicsTests.swift)
>   — convergence + parameter-sweep tests.

---

## 1. Why Isaac Sim policies don't "just work"

A policy exported from Isaac Lab encodes **four implicit assumptions** about
the simulator it was trained on. If any one of them is wrong, the robot will
either drift, oscillate, or fall over within ~1 s.

| # | Assumption | Where it lives |
|---|---|---|
| **1** | The 12 (or N) actions are emitted in **PhysX `dof_names` order**, not URDF / asset declaration order. | Isaac Sim articulation traversal |
| **2** | The observation is **`[lin_vel_b(3), ang_vel_b(3), projected_gravity_b(3), command(3), joint_pos_delta(N), joint_vel(N), prev_action(N)]`** in the same dof order. For 12-DOF quadrupeds this is 48 dims; for H1 it is 69 dims. | Isaac Lab `LocomotionVelocityRoughEnv` |
| **3** | Each joint runs a **PD position drive** with stiffness `Kp`, damping `Kd`, effort/velocity limits, and an **action scale** that is added to a **default standing pose**. | `spot_env.yaml` / `anymal_env.yaml` |
| **4** | The policy is queried **once per `decimation` physics steps**, with `physics_dt` matching what the network was trained on. | `policy_controller.py` |

The Spot regression was assumption **(1)**: the local Jolt simulator builds
joints in **leg-major** order (one full leg HAA→HFE→KFE before moving to the
next), but PhysX hands `dof_names` back **type-grouped by depth in the
kinematic chain** (all hx → all hy → all kn for Spot). Without a permutation,
each action ended up driving the wrong joint and the robot fell over instantly.

---

## 2. Joint orderings in this repo

### 2.1 Local Jolt simulator (sim order, leg-major)

Both ANYmal-C and Spot share the topology in
[`IsaacSwiftPhysics.mm`](../../IsaacSwift/IsaacSwiftPhysics.mm) and use the **same
indexing**:

```text
Index  0  1  2   3  4  5   6  7  8   9  10 11
Leg    LF/FL    RF/FR    LH/HL    RH/HR
DOF    HAA/hx HFE/hy KFE/kn  ×  4 legs
```

Concretely:

```
[ 0:LF_HAA, 1:LF_HFE, 2:LF_KFE,
  3:RF_HAA, 4:RF_HFE, 5:RF_KFE,
  6:LH_HAA, 7:LH_HFE, 8:LH_KFE,
  9:RH_HAA,10:RH_HFE,11:RH_KFE ]
```

(For Spot, replace `*_HAA / *_HFE / *_KFE` with `*_hx / *_hy / *_kn` and the
leg labels become lower-case `fl, fr, hl, hr`.)

### 2.2 Isaac Sim / PhysX dof_names (policy order)

PhysX traverses an articulation **breadth-first by joint depth**. At each
depth the order follows the joint declarations under that link. As a result,
for both ANYmal-C and Spot, `articulation.dof_names` is **type-grouped, leg
order LF / LH / RF / RH (ANYmal) or FL / FR / HL / HR (Spot)** at each depth.

| Robot | Policy order (`dof_names`) |
|---|---|
| ANYmal-C | `[LF_HAA, RF_HAA, LH_HAA, RH_HAA, LF_HFE, RF_HFE, LH_HFE, RH_HFE, LF_KFE, RF_KFE, LH_KFE, RH_KFE]` |
| Spot    | `[fl_hx, fr_hx, hl_hx, hr_hx, fl_hy, fr_hy, hl_hy, hr_hy, fl_kn, fr_kn, hl_kn, hr_kn]` |

> **Note on ANYmal leg ordering.** The USD declares joints LF / LH / RF / RH,
> and Isaac Lab's *default* `JointPositionAction` would produce that depth
> order. The bundled CoreML policy here was trained with a different
> ordering — type-grouped LF / RF / LH / RH at every depth (the same shape
> as Spot). Confirmed empirically by
> `anymalPolicyJointPermutationAndGainSweep` over a 600-point grid: only
> the LF-RF-LH-RH permutation keeps the robot upright.

> Verify on Linux with:
> ```python
> from isaacsim.core.prims import SingleArticulation
> robot = SingleArticulation(prim_path=...); robot.initialize()
> print(robot.dof_names)
> ```
> Or extract from the USD by depth-traversal of `PhysicsRevoluteJoint` prims.

### 2.3 The bridge: `simToPolicyJointPermutation`

Defined per robot in [`PolicyModel.swift`](../../IsaacSwift/PolicyModel.swift):

```swift
let perm: [Int]   // length 12, index by sim order, value is policy order
```

Used in **two places** inside `DemoPolicyActionProvider`:

1. `updateJointState(positionDeltas:velocities:)` — incoming sim-order
   joint state is permuted **into policy order** before being placed at
   `obs[12:24]` and `obs[24:36]`.
2. `currentActions(at:)` — outgoing policy actions are permuted **back into
   sim order** before being scaled and applied as motor targets. The
   policy-order copy is also kept so that the next observation's
   `previous_action` slot at `obs[36:48]` is consistent.

| Robot | `simToPolicyJointPermutation` |
|---|---|
| ANYmal-C | `[0, 4, 8,   2, 6, 10,   1, 5, 9,   3, 7, 11]` |
| Spot     | `[0, 4, 8,   1, 5, 9,    2, 6, 10,   3, 7, 11]` |
| H1       | `[0, 1, 2,   3, 7, 11, 15,   4, 8, 12, 16,   5, 6,   9, 13, 17,   10, 14, 18]` |

Both policies are type-grouped by joint depth, but their bundled exports do
not use the same leg order: ANYmal-C is LF-LH-RF-RH, while Spot is
FL-FR-HL-HR. Do not infer this from the local sim order; pin it from the
actual exported policy / Isaac Sim controller and lock it with tests.
This is locked in by:

- [`anymalSimToPolicyPermutationMatchesPhysXDofOrder()`](../../IsaacSwiftTests/IsaacSwiftTests.swift)
- [`spotSimToPolicyPermutationMatchesPhysXDofOrder()`](../../IsaacSwiftTests/IsaacSwiftTests.swift)
- [`h1SimToPolicyPermutationMatchesFlatTerrainDofOrder()`](../../IsaacSwiftTests/IsaacSwiftTests.swift)
- [`simToPolicyJointPermutationsAreValidBijections()`](../../IsaacSwiftTests/IsaacSwiftTests.swift)

---

## 3. Observation layout (48-dim Isaac Lab `LocomotionVelocityRoughEnv`)

Built by `DemoPolicyActionProvider.demoObservations(at:)`:

| Slice | Symbol | Source | Frame |
|---|---|---|---|
| `[0:3]`   | `base_lin_vel_b`     | `IsaacSwiftAnymalObservation.baseLinearVelocityBody` | base body |
| `[3:6]`   | `base_ang_vel_b`     | `…baseAngularVelocityBody` | base body |
| `[6:9]`   | `projected_gravity_b`| `…gravityDirectionBody`    | base body |
| `[9:12]`  | `velocity_command`   | `(vx, vy, wz)` from UI / config | world / heading frame |
| `[12:24]` | `joint_pos − default_pos` | sim, **permuted to policy order** | joint |
| `[24:36]` | `joint_vel`           | sim, **permuted to policy order** | joint |
| `[36:48]` | `previous_action`     | last network output (already in policy order) | joint |

> **Key invariants**
> - Indices 12–35 must be in **policy order**, not sim order.
> - Index 36–47 stores the **raw, unscaled** policy output, *not* the
>   default-pose-added joint target.
> - `projected_gravity_b` is `R_BI · (0, 0, −1)`, i.e. unit vector pointing
>   *along* gravity expressed in the base frame. Standing upright = `(0, 0, −1)`.

If a future policy uses a different observation layout (e.g. adds height
sensors, drops command), this is the function to edit.

---

## 4. Per-robot configuration tables

Two tables must agree:

1. `RobotConfig` in [`IsaacSwiftPhysics.mm`](../../IsaacSwift/IsaacSwiftPhysics.mm)
   — physical dimensions, masses, gains, default pose, spawn height,
   physics step.
2. `IsaacPolicyRuntimeConfiguration.<robot>` in
   [`PolicyModel.swift`](../../IsaacSwift/PolicyModel.swift) — physics step,
   decimation, action scale, default command, joint permutation.

The simulator's `recommendedActionScale` and `recommendedPhysicsTimeStep`
must match the runtime configuration, asserted by
`robotRuntimeConfigurationsMatchSimulatorDefaults()`.

### 4.1 Spot — `kSpotConfig` / `IsaacPolicyRuntimeConfiguration.spotFlat`

| Parameter | Value | Source |
|---|---|---|
| `physics_dt` | `1/500 s` | Isaac Lab Spot training |
| `decimation` | `10` (policy at 50 Hz) | `spot_env.yaml` |
| `kp` | **240** N·m/rad | tuned by sweep, ≈ Isaac Lab |
| `kd` | **12** N·m·s/rad | tuned by sweep |
| `max_torque` | **240** N·m | tuned by sweep |
| `action_scale` | **0.2** | matches `spot.py` `_action_scale = 0.2` |
| `default_command` | `(0.8, 0, 0)` m/s | UI default |
| `spawn_z` | `0.505 m` | `spot.py` `position=[0,0,0.7]` minus base offset |
| Default pose | `[+0.1,0.9,−1.5; −0.1,0.9,−1.5; +0.1,1.1,−1.5; −0.1,1.1,−1.5]` (FL,FR,HL,HR) | `spot_env.yaml` `init_state.joint_pos` |
| `simToPolicy` | `[0,4,8, 1,5,9, 2,6,10, 3,7,11]` | sweep + USD inspection |

### 4.2 ANYmal-C — `kAnymalConfig` / `IsaacPolicyRuntimeConfiguration.anymalC`

| Parameter | Value | Source |
|---|---|---|
| `physics_dt` | `1/200 s` | Isaac Lab ANYmal training |
| `decimation` | `4` (policy at 50 Hz) | Isaac Lab |
| `kp` / `kd` | fallback PD only | not used while `AnymalActuatorRunner` is installed |
| actuator `effort_limit` | **80** N·m | `ActuatorNetLSTM` effort limit |
| actuator `saturation_effort` | **120** N·m | `anymal_env.yaml` |
| `action_scale` | **0.5** | matches ANYmal controller `_action_scale = 0.5` |
| `default_command` | `(1.0, 0, 0)` m/s | app/test rollout default |
| `spawn_z` | **0.60 m** | `anymal_env.yaml` `init_state.pos` |
| Default pose | `[0,0.4,−0.8; 0,0.4,−0.8; 0,−0.4,0.8; 0,−0.4,0.8]` (LF,RF,LH,RH) | `anymal_env.yaml` |
| `simToPolicy` | `[0,4,8, 2,6,10, 1,5,9, 3,7,11]` | bundled export + sweep |

> **Why ANYmal differs from Spot.** Spot uses a direct PD target policy.
> ANYmal-C uses an `ActuatorNetLSTM` learned actuator. In Isaac Sim the
> policy action is converted to a target position, the actuator network sees
> `(target + default_joint_pos - joint_pos, joint_vel)` every physics step,
> and the resulting torque is applied as effort. The local Jolt path now
> follows that architecture: when `AnymalActuatorRunner` is installed, hinge
> position motors are disabled and the learned effort is applied directly as a
> parent/child torque pair. The simulator's PD gains are only the fallback
> path used when no actuator is installed.

### 4.3 Unitree H1 — `kH1Config` / `IsaacPolicyRuntimeConfiguration.h1Flat`

| Parameter | Value | Source |
|---|---|---|
| `physics_dt` | `1/200 s` | `h1_env.yaml` `sim.dt = 0.005` |
| `decimation` | `4` (policy at 50 Hz) | `h1_env.yaml` |
| `action_scale` | **0.5** | `H1FlatTerrainPolicy._action_scale = 0.5` / `h1_env.yaml` |
| Drive path | Jolt position motor, no H1 external-torque drive | Isaac Sim `ArticulationAction(joint_positions=...)` |
| Motor target smoothing | **0 s** | direct position targets are applied every physics step |
| `default_command` | `(1.0, 0, 0)` m/s | Isaac Sim H1 example forward command |
| `spawn_z` | `1.05 m` | Isaac Sim H1 example spawn position |
| Observation shape | 69 = base(12) + joint_pos(19) + joint_vel(19) + prev_action(19) | `H1FlatTerrainPolicy._compute_observation` |
| Action shape | 19 | H1 `robot.num_dof` |
| `simToPolicy` | `[0,1,2, 3,7,11,15, 4,8,12,16, 5,6, 9,13,17, 10,14,18]` | PhysX traversal order from the Isaac Sim H1 policy |

H1 is a 19-DOF direct position policy, not the quadruped 12-DOF layout. Do
not run it through the 48-observation quadruped path. The local simulator's
USD file order is:

```text
[left_hip_yaw, right_hip_yaw, torso,
 left_hip_roll, left_hip_pitch, left_knee, left_ankle,
 right_hip_roll, right_hip_pitch, right_knee, right_ankle,
 left_shoulder_pitch, right_shoulder_pitch,
 left_shoulder_roll, left_shoulder_yaw, left_elbow,
 right_shoulder_roll, right_shoulder_yaw, right_elbow]
```

The H1 policy order is PhysX breadth-first traversal:

```text
[left_hip_yaw, right_hip_yaw, torso,
 left_hip_roll, right_hip_roll,
 left_shoulder_pitch, right_shoulder_pitch,
 left_hip_pitch, right_hip_pitch,
 left_shoulder_roll, right_shoulder_roll,
 left_knee, right_knee,
 left_shoulder_yaw, right_shoulder_yaw,
 left_ankle, right_ankle,
 left_elbow, right_elbow]
```

If H1 appears to stand and sway without walking, check the ankle ranges first.
The ankles should move during a policy rollout, but useful walking only appears
when observation and action arrays are remapped to this traversal order.

H1 should spawn in the USD zero pose at `z = 1.05` and report
`current_joint_pos - default_pos` to the policy. The action target is then
`default_pos + action * 0.5`, applied through the Jolt position motor. Do not
add an H1-specific external torque drive unless a future regression proves the
position motor cannot hold the articulation; the current walking path passes
without an external upright-assist fallback.

### 4.4 Other Isaac-Lab values reproduced in the simulator

- **Self-collision off** (`Layers::kMoving` does not collide with itself in
  the broad-phase / object filter).
- **Gravity** = `(0, 0, −9.81)` m/s².
- **Hinge limits** ±π (wide enough that the PD motor doesn't chatter against
  them; Isaac Lab uses generous joint limits too).
- **Foot friction** ≈ 1.2; ground friction = 1.0.
- **Motor target smoothing** is per-robot/tuning dependent. For learned
  actuator paths, prefer matching Isaac Sim's per-physics-step actuator call
  before adding smoothing; smoothing can hide cadence or actuator-state bugs.

### 4.5 Policy actuator types matter

Before tuning gains, identify the controller type from the Isaac Sim robot
class and environment YAML:

| Type | What the network outputs | Runtime integration |
|---|---|---|
| Direct PD / position policy | normalized joint target deltas | scale by `action_scale`, add default pose, drive the simulator's PD motor |
| Learned actuator, feed-forward | target error / velocity features to a torque model | call the actuator every physics substep, apply torque, keep actuator inputs in policy order |
| Learned actuator, recurrent (`LSTM`/`GRU`) | torque plus hidden state | preserve hidden state across physics steps, reset it on episode reset, and do not recreate the runner per frame |

Checklist for recurrent actuator policies:

- Port both the policy and actuator model; a walking policy may depend on the
  actuator model even if the policy itself is feed-forward.
- Match the actuator input exactly. ANYmal's LSTM uses
  `target + default_joint_pos - joint_pos` and clips `joint_vel` to
  `[-20, 20]` before inference.
- Run the actuator at the physics rate, not the policy rate. Isaac Sim
  recomputes policy actions every `decimation` steps, but computes actuator
  torque on every physics step using the latest action.
- Keep action, joint position, joint velocity, and previous action arrays in
  the same policy order. A correct policy permutation with a wrong actuator
  ordering can look like a weak or frozen front-leg gait.
- Treat missing model outputs or inference errors as fatal during development.
  Returning zero actions/torques makes test/runtime differences hard to see.
- Unit-test a renderer-cadence loop separately from a pure fixed-step loop.
  The renderer should call into the same `PolicyPhysicsLoop`, not a UI-test
  only harness.
- Keep long walking tests opt-in until the simulator approximation is known
  to be stable. In this repo, set `ISAACSWIFT_ANYMAL_LONG_WALK_TESTS=1` to
  run the 12 s ANYmal checks; regular CI uses shorter boundedness tests.

---

## 5. The policy / physics tick

[`PolicyPhysicsLoop.step(at:)`](../../IsaacSwift/PolicyPhysicsLoop.swift) implements
the same control flow as Isaac Sim's `policy_controller.forward`:

```text
for each render frame:
    elapsed = frame_dt (capped to one decimation interval)
    obs    = sim.currentObservation()      // sim order
    provider.updateJointState(obs)         // → policy order
    provider.updateBaseFeedback(obs)
    raw    = provider.currentActions()     // CoreML inference if scheduler fires
                                           //   – obs in policy order
                                           //   – raw returned in sim order
    scaled = raw * action_scale            // applied as Δ from default pose
    sim.step(scaled, elapsed)              // EMA smoothed PD targets
                                           //   target_i = default_i + scaled_i
```

The simulator runs `floor(elapsed / physics_dt)` substeps per call (capped
to 32) so that wall-clock-driven render loops still produce deterministic
physics. `currentActions` is gated by `PolicyUpdateScheduler`, so the CoreML
network is only invoked once per `decimation` substeps.

---

## 6. Adding a new robot — checklist

This is the recommended workflow for porting, e.g., **ANYmal-D** or
**Unitree Go2** when their CoreML / TorchScript policies become available.

### 6.1 Gather Isaac Lab metadata

From the policy's `<robot>_env.yaml`:

- `physics_dt`, `decimation`
- `actuators.<group>.stiffness`, `damping`, `effort_limit`, `velocity_limit`
- `init_state.joint_pos` (default standing pose, regex → 12 floats)
- `_action_scale` from the robot controller (`<robot>.py`)
- Whether the policy is direct-PD, feed-forward actuator, or recurrent
  actuator (`ActuatorNetLSTM`, `GRU`, etc.).
- Any actuator-network preprocessing: position-error formula, velocity
  clipping, torque clipping, hidden-state shapes, and reset behavior.
- Observation layout (verify it matches the 48-dim slice table above; if
  not, edit `demoObservations`).

From the robot's USD (or PhysX in Isaac Sim):

- `articulation.dof_names` — copy the **exact list** in PhysX order.
- Spawn height (`position=[..., z]` in the controller, minus base half-height).

### 6.2 Add the simulator config

Edit [`IsaacSwiftPhysics.mm`](../../IsaacSwift/IsaacSwiftPhysics.mm):

1. Append a new `IsaacSwiftRobotKind` case in
   [`IsaacSwiftPhysics.h`](../../IsaacSwift/IsaacSwiftPhysics.h).
2. Add a `static const RobotConfig k<Robot>Config = { … }` filled with the
   values from §6.1.
3. Wire it through `ConfigForKind`.

### 6.3 Add the runtime config

Edit [`PolicyModel.swift`](../../IsaacSwift/PolicyModel.swift):

```swift
static let <robot> = IsaacPolicyRuntimeConfiguration(
    robotKind: .<robot>,
    physicsTimeStep: …,
    policyDecimation: …,
    actionScale: …,
    defaultCommand: SIMD3<Float>(…),
    simToPolicyJointPermutation: derivePermutation(
        simOrder:    [...],     // your leg-major sim ordering
        policyOrder: [...]      // dof_names from PhysX
    )
)
```

Where `derivePermutation` is conceptually:

```swift
simOrder.map { name in policyOrder.firstIndex(of: name)! }
```

Add a Swift Testing `@Test` that pins the permutation literal so future
refactors can't silently break it.

### 6.4 Add the renderer articulation profile

Edit [`Renderer+Articulation.swift`](../../IsaacSwift/Renderer+Articulation.swift):

- A `JointActionBinding` per joint (USD node path → action index).
- An `ArticulationTransformOverride` per body if your USD uses non-standard
  joint frames (extract them with `usdcat` or
  [`scripts/usdz/`](../../scripts/usdz/)).

### 6.5 Add tests before tuning

The bundled sweep test
[`spotPolicyJointPermutationAndGainSweep`](../../IsaacSwiftTests/IsaacSwiftPhysicsTests.swift)
is a template. Duplicate it for the new robot and add focused non-UI tests:

- The permutation literal is correct (all reasonable alternatives keep the
  robot upright; wrong ones tip it over within ~1.5 s).
- `kp / kd / max_torque / action_scale` produce a forward velocity within
  the commanded value's tolerance and `uprightZ > 0.95`.
- Recurrent actuator state is reset on `PolicyPhysicsLoop.reset()`.
- The renderer-cadence path (`controlInterval: 1/60`) uses the same loop and
  stays within the fixed-step result envelope.
- Long-horizon walking tests are useful during tuning, but they should be
  gated by an environment variable until they are stable across local and
  device runs.

Run with:

```bash
./scripts/test-unit.sh                  # default — quick sweep
ISAACSWIFT_SPOT_PERM_SWEEP_FULL=1 \
  ./scripts/test-unit.sh                # broader grid
```

### 6.6 Lock in a strict end-to-end test

Mirror
[`spotCoreMLPolicyWalksForwardOnFlatGround`](../../IsaacSwiftTests/IsaacSwiftPhysicsTests.swift):

```swift
#expect(stats.minBaseUprightZ > 0.95)
#expect(stats.minBaseZ > 0.35)              // robot didn't collapse
#expect(stats.endBasePos.x > command.x * seconds * 0.4)  // ≥ 40% of nominal
```

If these pass, the policy is production-ready in this app.

---

## 7. Final verification runbook

Before calling a policy/physics fix complete, run these checks in this order:

1. Remove temporary debug gates and skips.
   - Search for `if true { return }`, ad hoc env guards, and debug prints such
     as `H1DBG` or `SWEEP`.
   - Keep useful permanent assertions, such as ankle-range checks for H1 or
     front-leg swing checks for quadrupeds.
2. Lock the joint order.
   - Add or update the static permutation test in
     [`IsaacSwiftTests.swift`](../../IsaacSwiftTests/IsaacSwiftTests.swift).
   - Run the focused static test before spending time on long rollouts.
3. Confirm passive or stand-still stability.
   - Run the robot's zero-action headless test when raw zero actions are a
     valid hold-pose command.
   - For H1, raw zero action is not the release invariant; run the CoreML
     zero-command test instead because the humanoid stand-still behavior is
     policy-controlled.
   - It should stay finite, keep reasonable base height/uprightness, and not
     build extreme joint velocity before policy feedback is involved.
   - Prefer matching USD/Isaac solver, mass, inertia, drive, and contact
     settings before adding external stabilization. Avoid stacking workaround
     types; treat a Jolt-only stabilization fallback as a last resort that
     requires a documented regression and focused H1 test coverage.
4. Confirm the policy rollout.
   - Run the focused CoreML policy test for the robot.
   - For H1, require at least 1 m forward progress over 10 s and verify both
     ankle joints have non-trivial range.
   - For Spot/ANYmal, require forward progress plus upright/base-height checks.
5. Restore broader coverage.
   - Re-enable Spot, ANYmal, and diagnostics that were skipped during H1-only
     debugging.
   - Add a renderer-cadence regression when a symptom only appears on screen.
   - Run `./scripts/agent-ci.sh` without arguments. This is the normal build
     plus smoke coverage path for agents.
6. Only use full sweeps when the focused tests fail.
   - `ISAACSWIFT_SPOT_PERM_SWEEP_FULL=1 ./scripts/test-unit.sh`
   - Add equivalent robot-specific sweep env gates when a new policy needs
     broad tuning.
7. If a renderer symptom remains, verify after the headless tests pass.
   - The renderer should consume `PolicyPhysicsLoop` output, not a separate
     UI-only stepping path.
   - `basePositionWorld` should describe the USD base link origin used by the
     renderer, not the inertial COM. Robots with non-zero COM offsets, such as
     H1 pelvis, will otherwise look slightly sunk into or floating above the
     ground even when the headless rollout is healthy.
   - UI tests stay launch/smoke-level; walking, cadence, and policy correctness
     belong in unit/headless tests.

Do not finish with only a visual check. A robot that "looks like it moves" may
still have incorrect policy order, previous-action order, actuator cadence, or
contact feedback.

---

## 8. Symptoms ↔ likely root causes

| Symptom | Likely cause | Where to look |
|---|---|---|
| Falls over within 1 s, joints look like they're "in someone else's leg" | Joint permutation wrong | `simToPolicyJointPermutation`, `dof_names` |
| Walks but slow drift / pitches forward | `default_joint_pos` mismatch with training | `RobotConfig.defaults`, `init_state.joint_pos` |
| Walks but oscillates / chatters | `kp` too high or `kd` too low; or PD loop runs at wrong rate | `RobotConfig.kp/kd`, `physicsTimeStep`, `policyDecimation` |
| Base sinks under gravity | `kp / max_torque` too low for the masses | `RobotConfig.{kp, maxTorque, *Mass}` |
| Robot drops at spawn before policy stabilizes | `spawn_z` too high; or default pose feet not on the ground | `RobotConfig.spawnZ`, `defaults` |
| Base "teleports" between frames | `step:` accumulator not draining; check `elapsed` source | `PolicyPhysicsLoop.step(at:)` |
| Outputs `NaN` / huge velocities | Hinge angles outside limits, or actions not clipped | hinge limits, `action_scale` |
| Walks fine zero-action but tips with command | Observation slice ordering bug | `demoObservations`, gravity sign |
| H1 stands and sways but barely translates | H1 policy order is still USD file order, not PhysX BFS traversal | `IsaacPolicyRuntimeConfiguration.h1Flat.simToPolicyJointPermutation` |
| Ankles move but no propulsion | Actions/observations reach ankles, but contact or policy order is wrong | ankle range checks, H1 permutation test, foot collision/friction |
| H1 keeps height but barely translates | H1 is using a chain/regex action order instead of PhysX breadth-first policy order | `h1SimToPolicyPermutationMatchesFlatTerrainDofOrder()` |
| H1 walks but feet look buried | Jolt contact slop or rounded foot box is too large for the 2.4 cm-thick H1 foot | H1 `BoxShape(..., 0.0f)`, `mPenetrationSlop`, `mSpeculativeContactDistance` |
| Walks correctly but feet look slightly sunk into the ground | Renderer base pose is aligned to inertial COM instead of USD link origin | `IsaacSwiftAnymalSimulator.currentObservation().basePositionWorld`, `Renderer.applyPhysicsBasePose` |

---

## 9. References

- Isaac Lab `LocomotionVelocityRoughEnv` — observation / reward layout.
- [`IsaacSim/source/extensions/isaacsim.robot.policy.examples/.../spot.py`](../../IsaacSim/source/extensions/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/spot.py)
  — reference forward / observation pipeline.
- [`IsaacSim/source/extensions/isaacsim.robot.policy.examples/.../anymal.py`](../../IsaacSim/source/extensions/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/anymal.py)
  — same for ANYmal.
- [`IsaacSim/source/extensions/isaacsim.robot.policy.examples/.../policy_controller.py`](../../IsaacSim/source/extensions/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/controllers/policy_controller.py)
  — PD gain / effort-limit pipeline that we reproduce in `IsaacSwiftPhysics.mm`.
