---
name: isaacswift-asset-pipeline
description: Use this skill when packaging, regenerating, validating, or debugging IsaacSwift robot USDZ assets from Isaac Sim robot asset sources, including ANYmal-C, Spot, Unitree Go2, and Unitree H1.
---

# Asset Pipeline

This project bundles 3D assets directly into the `Metal 4` app `IsaacSwift`.

## Preconditions

- This project is `Metal 4` only.
- Verify assets against `iphoneos` and real devices, not the simulator.
- Use `./scripts/build-device.sh` for device builds.

## Source Assets

The preferred inputs are the `USD` robot assets shipped in the Isaac Sim robot asset pack.

Examples:

- `isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/ANYbotics/anymal_c`
- `isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/Unitree/Go2`
- `isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/Unitree/H1`

## Output Location

Built artifacts used by the app belong under `IsaacSwift/RobotAssets/<asset-name>/`.
Those generated runtime assets are local artifacts, not committed source files.
Source assets stay out of the app bundle; package scripts should read directly from `isaac-sim-assets-robots_and_sensors-5.1.0/...`.

Examples:

- `IsaacSwift/RobotAssets/anymal_c/`
- `IsaacSwift/RobotAssets/spot/`
- `IsaacSwift/RobotAssets/go2/`
- `IsaacSwift/RobotAssets/h1/`

## Required Files

At minimum you need these two categories:

- The root `USD` or `USDZ` file
- Any texture, payload, or material files referenced by that root `USD`

Examples:

- `anymal_c.usdz`
- `Props/instanceable_meshes.usd`
- `Props/materials/*.jpg`

## Files To Exclude

Do not include the following unless you have a specific reason:

- `.thumbs/`
- Auxiliary files used only for testing or documentation
- `legacy/` trees that duplicate current texture names

In particular, if both `legacy/materials/*.jpg` and `Props/materials/*.jpg` are bundled, Xcode can fail because of resource name collisions.

## Packaging

### ANYmal C

```bash
make anymal-usdz
```

The package script builds `anymal_c.usdz` from the Isaac Sim asset pack at `ANYbotics/anymal_c/` and leaves only that `usdz` under `IsaacSwift/RobotAssets/anymal_c/`.
Keep USDZ helper scripts under `scripts/usdz/`.
To run it directly, use `./scripts/usdz/package_anymal_usdz.sh`.
During packaging, the script text-expands `Props/instanceable_meshes.usd` and rewrites `OmniPBR` materials into `UsdPreviewSurface + UsdUVTexture`, which Apple tooling handles more reliably.

### Spot

```bash
make spot-usdz
```

The package script builds `spot.usdz` from the Isaac Sim asset pack at `BostonDynamics/spot/`.
`spot.usd` also uses `OmniPBR`, so apply the same `UsdPreviewSurface + UsdUVTexture` rewrite used for ANYmal.

### Go2

```bash
make go2-usdz
```

The package script builds `go2.usdz` from the Isaac Sim asset pack at `Unitree/Go2/`.
Go2 does not ship image textures for its main materials. It uses `diffuse_color_constant` values in the USD, so the renderer must provide a solid-color fallback from the model definition.

### H1

```bash
make h1-usdz
```

The package script builds `h1.usdz` from the Isaac Sim asset pack at `Unitree/H1/`.
Keep the root `h1.usd` plus the payload files (`base.usda`, `physics*.usda`, `geometries.usd`, and hand payloads) together so the default `Physx_minimal` variant resolves correctly.

## Loader Assumptions

The renderer/loader is expected to:

- Read `USDZ` files from `RobotAssets/...` inside the bundle
- Expand the `USDZ` into a temporary location before handing it to `ModelIO`
- Convert `MDLAsset -> MTKMesh` with `ModelIO` and `MetalKit`
- Load base-color textures when they exist
- Infer texture file names from material names so `Props/materials/*.jpg` can still resolve
- Fall back to a solid default texture when no image texture exists

## Naming

- Keep asset names aligned with the Isaac Sim source asset names.
- Keep source and packaged asset names consistent.
- Example: `.../ANYbotics/anymal_c/anymal_c.usd` -> `IsaacSwift/RobotAssets/anymal_c/anymal_c.usdz`

## Validation Checklist

When adding or updating an asset, verify:

- The root `USD` or `USDZ` exists
- Referenced textures and payloads are included
- A clean checkout can regenerate assets with `make usdz`
- `IsaacSwift/RobotAssets/anymal_c/` does not contain anything except `anymal_c.usdz`
- `.thumbs` are excluded
- `legacy` textures are not duplicated alongside current textures
- Validation uses `./scripts/build-device.sh` for `iphoneos`

## Notes

- Simulator builds are out of scope for this `Metal 4` project.
- If a `USD` references other `USD` files, preserve enough of the original directory structure for the relative paths to remain valid.
- For ANYmal C, ship only `anymal_c.usdz` in the bundle to avoid relative-reference breakage from raw `usdc`.
- Xcode file-system synchronized groups may flatten resource names. Watch for duplicate image names.

## Policy Model

- Compiled policy models should live in the app as:
  - `PolicyModels/spot_policy.mlmodelc`
  - `PolicyModels/anymal_policy.mlmodelc`
  - `PolicyModels/h1_policy.mlmodelc`
- Policy sources are fetched locally from NVIDIA into `isaac_policy_sources/`.
- Build conversion tooling once with `make policy-tooling`.
- Fetch published Spot and ANYmal sources with `make fetch-policies`.
- Build all policy bundles with `make policy-model`.
- Build only the ANYmal bundle with `make anymal-policy-model`.
- Build only the H1 bundle with `make h1-policy-model`.
- Go2 currently reuses the Spot policy as a temporary placeholder.
