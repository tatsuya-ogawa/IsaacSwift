#!/bin/zsh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

policy_source_root="${POLICY_SOURCE_ROOT:-isaac_policy_sources}"
pretrained_policy_raw_base="${PRETRAINED_POLICY_RAW_BASE:-https://raw.githubusercontent.com/tatsuya-ogawa/IsaacSim_pretrained_models/main}"
go2_backflip_policy_raw_url="${GO2_BACKFLIP_POLICY_RAW_URL:-https://raw.githubusercontent.com/tatsuya-ogawa/IsaacSim-go2-backflip/main/assets/policy.pt}"
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

fetch_h1() {
  local dest_dir="$policy_source_root/H1_Policies"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/H1_Policies/h1_policy.pt" \
           "$dest_dir/h1_policy.pt"
  download "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Samples/Policies/H1_Policies/h1_env.yaml" \
           "$dest_dir/h1_env.yaml"
  echo "Fetched H1 policy sources into $dest_dir"
}

fetch_anymal_rough() {
  local dest_dir="$policy_source_root/Anymal_Policies"
  download "$pretrained_policy_raw_base/anymal_c_rough_direct/exported/policy.pt" \
           "$dest_dir/anymal_rough_policy.pt"
  download "$pretrained_policy_raw_base/anymal_c_rough_direct/params/env.yaml" \
           "$dest_dir/anymal_rough_env.yaml"
  download "$pretrained_policy_raw_base/anymal_c_rough_direct/params/agent.yaml" \
           "$dest_dir/anymal_rough_agent.yaml"
  echo "Fetched ANYmal rough policy sources into $dest_dir"
}

fetch_go2() {
  local dest_dir="$policy_source_root/Go2_Policies"
  download "$pretrained_policy_raw_base/unitree_go2_flat/exported/policy.pt" \
           "$dest_dir/go2_policy.pt"
  download "$pretrained_policy_raw_base/unitree_go2_flat/params/env.yaml" \
           "$dest_dir/go2_env.yaml"
  download "$pretrained_policy_raw_base/unitree_go2_flat/params/agent.yaml" \
           "$dest_dir/go2_agent.yaml"
  echo "Fetched Go2 flat policy sources into $dest_dir"
}

fetch_go2_rough() {
  local dest_dir="$policy_source_root/Go2_Policies"
  download "$pretrained_policy_raw_base/unitree_go2_rough/exported/policy.pt" \
           "$dest_dir/go2_rough_policy.pt"
  download "$pretrained_policy_raw_base/unitree_go2_rough/params/env.yaml" \
           "$dest_dir/go2_rough_env.yaml"
  download "$pretrained_policy_raw_base/unitree_go2_rough/params/agent.yaml" \
           "$dest_dir/go2_rough_agent.yaml"
  echo "Fetched Go2 rough policy sources into $dest_dir"
}

fetch_go2_backflip() {
  local dest_dir="$policy_source_root/Go2_Policies"
  download "$go2_backflip_policy_raw_url" \
           "$dest_dir/go2_backflip_policy.pt"
  echo "Fetched Go2 backflip policy source into $dest_dir"
}

case "$selection" in
  spot)
    fetch_spot
    ;;
  anymal)
    fetch_anymal
    ;;
  anymal_rough|anymal-rough)
    fetch_anymal_rough
    ;;
  h1)
    fetch_h1
    ;;
  go2|go2_flat|go2-flat)
    fetch_go2
    ;;
  go2_rough|go2-rough)
    fetch_go2_rough
    ;;
  go2_backflip|go2-backflip)
    fetch_go2_backflip
    ;;
  all)
    fetch_spot
    fetch_anymal
    fetch_anymal_rough
    fetch_h1
    fetch_go2
    fetch_go2_rough
    fetch_go2_backflip
    ;;
  *)
    echo "Usage: $0 [spot|anymal|anymal_rough|h1|go2|go2_rough|go2_backflip|all]" >&2
    exit 1
    ;;
esac
