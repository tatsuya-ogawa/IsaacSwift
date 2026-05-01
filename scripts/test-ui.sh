#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/.."
./scripts/ensure-robot-assets.sh
./scripts/ensure-policy-model.sh
source ./scripts/test-common.sh

VERBOSE=$(parse_verbose_flag "$@")

set +e
DESTINATION=$(resolve_ios_device_destination)
DESTINATION_STATUS=$?
set -e

if (( DESTINATION_STATUS != 0 )); then
  echo "Skipping UI Tests (iOS Device): no eligible physical iOS device."
  exit 0
fi
[[ -n "$DESTINATION" ]] || exit 1

if run_xcode_test "UI Tests (iOS Device)" "IsaacSwiftUITests" "IsaacSwiftTests" "$VERBOSE" "$DESTINATION"; then
  TEST_STATUS=0
else
  TEST_STATUS=$?
fi

if (( TEST_STATUS == 70 )); then
  echo "Skipping UI Tests (iOS Device): destination is not currently available."
  exit 0
fi

exit "$TEST_STATUS"
