#!/bin/zsh
# Packages the Unitree Go2 USD assets shipped with Isaac Sim 5.1 into a
# self-contained `go2.usdz` consumed by the iOS renderer.
#
# Usage: ./scripts/usdz/package_go2_usdz.sh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

source_dir="isaac-sim-assets-robots_and_sensors-5.1.0/Assets/Isaac/5.1/Isaac/Robots/Unitree/Go2"
output_dir="IsaacSwift/RobotAssets/go2"
output_file="$output_dir/go2.usdz"

for required in go2.usd go2/configuration.usd; do
  if [[ ! -e "$source_dir/$required" ]]; then
    echo "Missing source asset: $source_dir/$required" >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
rm -f "$output_file"

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/go2_usdz.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT

prepared_source_dir="$tmp_dir/prepared_source"
mkdir -p "$prepared_source_dir"
cp -R "$source_dir"/. "$prepared_source_dir"/

(
  cd "$prepared_source_dir"
  usdzip -r "$tmp_dir/go2.usdz" go2.usd go2
)

mv "$tmp_dir/go2.usdz" "$output_file"
echo "Wrote $output_file"
