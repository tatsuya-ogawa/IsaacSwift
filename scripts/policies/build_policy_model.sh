#!/bin/zsh
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
cd "$repo_root"

policy_variant="${1:-spot}"
policy_source_root="${POLICY_SOURCE_ROOT:-isaac_policy_sources}"
policy_venv="${POLICY_BUILD_VENV:-.venv-policy-build}"
python_bin="$policy_venv/bin/python"

if [[ ! -x "$python_bin" ]]; then
  echo "Missing policy conversion environment: $python_bin" >&2
  echo "Run: make policy-tooling" >&2
  exit 1
fi

case "$policy_variant" in
  spot)
    policy_pt="$policy_source_root/Spot_Policies/spot_policy.pt"
    package_name="spot_policy.mlpackage"
    policy_dest_dir="${POLICY_MODEL_DEST_DIR:-PolicyModels/spot_policy.mlmodelc}"
    input_shape="1,48"
    ;;
  anymal)
    policy_pt="$policy_source_root/Anymal_Policies/anymal_policy.pt"
    package_name="anymal_policy.mlpackage"
    policy_dest_dir="${POLICY_MODEL_DEST_DIR:-PolicyModels/anymal_policy.mlmodelc}"
    input_shape="1,48"
    ;;
  h1)
    policy_pt="$policy_source_root/H1_Policies/h1_policy.pt"
    package_name="h1_policy.mlpackage"
    policy_dest_dir="${POLICY_MODEL_DEST_DIR:-PolicyModels/h1_policy.mlmodelc}"
    input_shape="1,69"
    ;;
  *)
    echo "Unsupported policy variant: $policy_variant" >&2
    echo "Supported variants: spot, anymal, h1" >&2
    exit 1
    ;;
esac

if [[ ! -f "$policy_pt" ]]; then
  echo "Missing policy source: $policy_pt" >&2
  echo "Run: make fetch-policies" >&2
  exit 1
fi

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/isaac_policy_build.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT

mlpackage_path="$tmp_dir/$package_name"
compiled_output_dir="$tmp_dir/compiled"

"$python_bin" scripts/policies/convert_torchscript_to_coreml.py \
  --input "$policy_pt" \
  --output "$mlpackage_path" \
  --input-shape "$input_shape"

mkdir -p "$compiled_output_dir"
xcrun coremlcompiler compile "$mlpackage_path" "$compiled_output_dir" >/dev/null

mkdir -p "$(dirname "$policy_dest_dir")"
rm -rf "$policy_dest_dir"
mv "$compiled_output_dir/${package_name%.mlpackage}.mlmodelc" "$policy_dest_dir"

echo "Built $policy_dest_dir"
