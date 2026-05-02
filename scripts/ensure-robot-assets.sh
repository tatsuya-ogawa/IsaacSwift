#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/.."

required_assets=(
  "IsaacSwift/RobotAssets/anymal_c/anymal_c.usdz"
  "IsaacSwift/RobotAssets/spot/spot.usdz"
  "IsaacSwift/RobotAssets/go2/go2.usdz"
  "IsaacSwift/RobotAssets/h1/h1.usdz"
)

missing_assets=()
for asset_path in "${required_assets[@]}"; do
  [[ -f "$asset_path" ]] || missing_assets+=("$asset_path")
done

if (( ${#missing_assets[@]} == 0 )); then
  exit 0
fi

asset_pack_root="isaac-sim-assets-robots_and_sensors-5.1.0"

echo "Robot assets are missing." >&2
if [[ -d "$asset_pack_root" ]]; then
  echo "Generate them with: make usdz" >&2
else
  echo "Fetch the Isaac Sim robot asset pack into: $asset_pack_root" >&2
  echo "Then generate runtime assets with: make usdz" >&2
fi

for asset_path in "${missing_assets[@]}"; do
  echo "Missing: $asset_path" >&2
done

exit 1
