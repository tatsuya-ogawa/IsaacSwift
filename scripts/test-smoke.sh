#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/.."
./scripts/ensure-robot-assets.sh
./scripts/ensure-policy-model.sh
source ./scripts/test-common.sh

VERBOSE=$(parse_verbose_flag "$@")
DESTINATION=$(resolve_designed_for_ipad_destination) || exit $?
[[ -n "$DESTINATION" ]] || exit 1

run_xcode_test "Smoke Unit Tests (My Mac)" "IsaacSwiftTests" "IsaacSwiftUITests" "$VERBOSE" "$DESTINATION"
