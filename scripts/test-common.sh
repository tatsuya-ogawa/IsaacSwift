#!/bin/zsh

resolve_designed_for_ipad_destination() {
  if [[ -n "${ISAACSWIFT_TEST_DESTINATION:-}" ]]; then
    echo "$ISAACSWIFT_TEST_DESTINATION"
    return
  fi

  local destination_id
  destination_id=$(xcodebuild -showdestinations -scheme IsaacSwift -project IsaacSwift.xcodeproj 2>/dev/null | grep "variant:Designed for \[iPad,iPhone\]" | head -n 1 | sed 's/.*id:\([^,]*\).*/\1/' | xargs)

  if [[ -z "$destination_id" ]]; then
    echo "Error: Could not find destination 'Designed for [iPad,iPhone]'" >&2
    exit 1
  fi

  echo "id=$destination_id"
}

resolve_ios_device_destination() {
  if [[ -n "${ISAACSWIFT_UI_TEST_DESTINATION:-}" ]]; then
    echo "$ISAACSWIFT_UI_TEST_DESTINATION"
    return
  fi

  local destinations
  destinations=$(xcodebuild -showdestinations -scheme IsaacSwift -project IsaacSwift.xcodeproj 2>/dev/null)

  local destination_id
  destination_id=$(print -r -- "$destinations" | awk '/Available destinations/{available=1; next} /Ineligible destinations/{available=0} available' | grep '{ platform:iOS,' | grep -vi 'placeholder' | head -n 1 | sed 's/.*id:\([^,]*\).*/\1/' | xargs)

  if [[ -z "$destination_id" ]]; then
    echo "Error: Could not find an eligible physical iOS device for UI tests." >&2
    print -r -- "$destinations" | awk '/Ineligible destinations/{ineligible=1; next} ineligible && /platform:iOS/' >&2
    echo "Set ISAACSWIFT_UI_TEST_DESTINATION='id=<device-id>' to override." >&2
    exit 1
  fi

  echo "id=$destination_id"
}

run_xcode_test() {
  local label="$1"
  local only_testing="$2"
  local skip_testing="$3"
  local verbose="$4"
  local destination="$5"

  local -a command
  command=(
    xcodebuild test
    -project IsaacSwift.xcodeproj
    -scheme IsaacSwift
    -destination "$destination"
    -destination-timeout 5
    -derivedDataPath .derivedData
    CODE_SIGNING_ALLOWED=NO
  )

  if [[ -n "$only_testing" ]]; then
    command+=(-only-testing:"$only_testing")
  fi
  if [[ -n "$skip_testing" ]]; then
    command+=(-skip-testing:"$skip_testing")
  fi

  if (( verbose )); then
    "${command[@]}"
    return
  fi

  echo "Running ${label}..."
  set +e
  local output
  output=$("${command[@]}" 2>&1)
  local test_status=$?
  set -e

  print -r -- "$output" | grep -E '^(✔|✘|◇ (Suite|Test run))|^\*\* TEST|^error:|xcodebuild: error:|:[0-9]+:[0-9]+: error:| error: error:|FAILED|SUCCEEDED' || true
  return "$test_status"
}

parse_verbose_flag() {
  local verbose=0
  for arg in "$@"; do
    case "$arg" in
      --verbose|-v) verbose=1 ;;
    esac
  done
  echo "$verbose"
}
