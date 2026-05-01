#!/bin/zsh
# Packages the ANYmal-C USD assets shipped with Isaac Sim 5.1 into a single
# self-contained `anymal_c.usdz` consumed by the iOS renderer.
#
# Usage: ./scripts/usdz/package_anymal_usdz.sh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

source_dir="isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/ANYbotics/anymal_c"
output_dir="IsaacSwift/RobotAssets/anymal_c"
output_file="$output_dir/anymal_c.usdz"

for required in anymal_c.usd config.yaml Props configuration; do
  if [[ ! -e "$source_dir/$required" ]]; then
    echo "Missing source asset: $source_dir/$required" >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
rm -f "$output_file"

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/anymal_usdz.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT

prepared_source_dir="$tmp_dir/prepared_source"
mkdir -p "$prepared_source_dir"
cp -R "$source_dir"/. "$prepared_source_dir"/
find "$prepared_source_dir" -name .thumbs -type d -prune -exec rm -rf {} +
rm -rf "$prepared_source_dir/legacy"

# ANYmal's mesh USD uses OmniPBR.mdl shaders. Model I/O does not interpret
# those reliably, so convert the material definitions to UsdPreviewSurface
# while keeping the original source tree untouched.
usdcat "$source_dir/Props/instanceable_meshes.usd" > "$prepared_source_dir/Props/instanceable_meshes.usd"
python3 scripts/usdz/rewrite_anymal_preview_surface.py \
  "$prepared_source_dir/Props/instanceable_meshes.usd" \
  "$prepared_source_dir/Props/instanceable_meshes.usd"

(
  cd "$prepared_source_dir"
  usdzip -r "$tmp_dir/anymal_c.usdz" anymal_c.usd config.yaml Props configuration
)

mv "$tmp_dir/anymal_c.usdz" "$output_file"
echo "Wrote $output_file"
