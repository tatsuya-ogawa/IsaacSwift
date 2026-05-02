#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/.."

required_policies=(
  "PolicyModels/spot_policy.mlmodelc"
  "PolicyModels/anymal_policy.mlmodelc"
  "PolicyModels/h1_policy.mlmodelc"
)

missing_policies=()
for policy_path in "${required_policies[@]}"; do
  [[ -d "$policy_path" ]] || missing_policies+=("$policy_path")
done

if (( ${#missing_policies[@]} == 0 )); then
  exit 0
fi

echo "Policy models are missing." >&2

if [[ -d "isaac_policy_sources" ]]; then
  echo "Generate them with: make policy-model" >&2
else
  echo "Fetch policy sources first with: make fetch-policies" >&2
  echo "Install conversion tooling with: make policy-tooling" >&2
  echo "Then generate them with: make policy-model" >&2
fi

for policy_path in "${missing_policies[@]}"; do
  echo "Missing: $policy_path" >&2
done

exit 1
