#!/bin/zsh
# Packages the Spot USD assets shipped with Isaac Sim 5.1 into a single
# self-contained `spot.usdz` consumed by the iOS renderer.
#
# Usage: ./scripts/usdz/package_spot_usdz.sh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

source_dir="isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/BostonDynamics/spot"
output_dir="IsaacSwift/RobotAssets/spot"
output_file="$output_dir/spot.usdz"

for required in spot.usd materials configuration; do
  if [[ ! -e "$source_dir/$required" ]]; then
    echo "Missing source asset: $source_dir/$required" >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
rm -f "$output_file"

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/spot_usdz.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT

prepared_source_dir="$tmp_dir/prepared_source"
mkdir -p "$prepared_source_dir"
cp -R "$source_dir"/. "$prepared_source_dir"/

# Spot's `spot.usd` declares its 28 materials with OmniPBR.mdl shaders, which
# Model I/O cannot interpret. Run the same MDL → UsdPreviewSurface rewrite
# we use for ANYmal so iOS pulls in the diffuse textures correctly.
usdcat "$source_dir/spot.usd" > "$prepared_source_dir/spot.usd"
python3 scripts/usdz/rewrite_anymal_preview_surface.py \
  "$prepared_source_dir/spot.usd" \
  "$prepared_source_dir/spot.usd"

(
  cd "$prepared_source_dir"
  usdzip -r "$tmp_dir/spot.usdz" spot.usd materials configuration
)

mv "$tmp_dir/spot.usdz" "$output_file"
echo "Wrote $output_file"
