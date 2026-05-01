#!/bin/zsh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

policy_source_root="${POLICY_SOURCE_ROOT:-isaac_policy_sources}"
selection="${1:-all}"

download() {
  local url="$1"
  local output_path="$2"
  mkdir -p "$(dirname "$output_path")"
  curl -L --fail --silent --show-error "$url" -o "$output_path"
}

fetch_spot() {
  local dest_dir="$policy_source_root/Spot_Policies"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Spot_Policies/spot_policy.pt" \
           "$dest_dir/spot_policy.pt"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Spot_Policies/spot_env.yaml" \
           "$dest_dir/spot_env.yaml"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Spot_Policies/agent.yaml" \
           "$dest_dir/agent.yaml"
  echo "Fetched Spot policy sources into $dest_dir"
}

fetch_anymal() {
  local dest_dir="$policy_source_root/Anymal_Policies"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Anymal_Policies/anymal_policy.pt" \
           "$dest_dir/anymal_policy.pt"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Anymal_Policies/sea_net_jit2.pt" \
           "$dest_dir/sea_net_jit2.pt"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Anymal_Policies/anymal_env.yaml" \
           "$dest_dir/anymal_env.yaml"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/Anymal_Policies/agent.yaml" \
           "$dest_dir/agent.yaml"
  echo "Fetched ANYmal policy sources into $dest_dir"
}

case "$selection" in
  spot)
    fetch_spot
    ;;
  anymal)
    fetch_anymal
    ;;
  all)
    fetch_spot
    fetch_anymal
    ;;
  *)
    echo "Usage: $0 [spot|anymal|all]" >&2
    exit 1
    ;;
esac
