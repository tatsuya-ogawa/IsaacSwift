#!/bin/zsh
# Packages the Unitree H1 USD assets shipped with Isaac Sim 5.1 into a
# self-contained `h1.usdz` consumed by the iOS renderer.
#
# Usage: ./scripts/usdz/package_h1_usdz.sh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

source_dir="isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/Unitree/H1"
output_dir="IsaacSwift/RobotAssets/h1"
output_file="$output_dir/h1.usdz"

required_files=(
  h1.usd
  payloads/base.usda
  payloads/physics.usda
  payloads/physics_45.usda
  payloads/physics_minimal.usda
  payloads/geometries.usd
  payloads/h1_hand_left.usd
  payloads/h1_hand_right.usd
)

for required in "${required_files[@]}"; do
  if [[ ! -e "$source_dir/$required" ]]; then
    echo "Missing source asset: $source_dir/$required" >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
rm -f "$output_file"

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/h1_usdz.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT

prepared_source_dir="$tmp_dir/prepared_source"
mkdir -p "$prepared_source_dir/payloads"

cp "$source_dir/h1.usd" "$prepared_source_dir/h1.usd"
for payload in "${required_files[@]:1}"; do
  cp "$source_dir/$payload" "$prepared_source_dir/$payload"
done

(
  cd "$prepared_source_dir"
  usdzip -r "$tmp_dir/h1.usdz" h1.usd payloads
)

mv "$tmp_dir/h1.usdz" "$output_file"
echo "Wrote $output_file"
