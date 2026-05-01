#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/.."

./scripts/ensure-robot-assets.sh
./scripts/ensure-policy-model.sh

# Default: quiet mode (minimal output for CI/agent use)
# Use --verbose or -v for full xcodebuild output
VERBOSE=0
for arg in "$@"; do
  case "$arg" in
    --verbose|-v) VERBOSE=1 ;;
    *sim*) echo "This project is Metal 4 only. Simulator builds are not supported." >&2; exit 1 ;;
  esac
done

xcodebuild -downloadComponent MetalToolchain >/dev/null 2>&1 || true

toolchain_root=""
for candidate in /var/run/com.apple.security.cryptexd/mnt/com.apple.MobileAsset.MetalToolchain-*/Metal.xctoolchain; do
  if [[ -d "$candidate" ]]; then
    toolchain_root="$candidate"
    break
  fi
done

if [[ -z "$toolchain_root" ]]; then
  echo "Metal Toolchain is not available. Run: xcodebuild -downloadComponent MetalToolchain" >&2
  exit 1
fi

toolchain_id=$(/usr/libexec/PlistBuddy -c "Print Identifier" "$toolchain_root/ToolchainInfo.plist")
metal_bin="$toolchain_root/usr/bin/metal"

if [[ ! -x "$metal_bin" ]]; then
  echo "Metal Toolchain binary is missing at: $metal_bin" >&2
  exit 1
fi

if ! "$metal_bin" -v >/dev/null 2>&1; then
  echo "Metal Toolchain exists but is not executable in the current environment." >&2
  exit 1
fi

export TOOLCHAINS="$toolchain_id"

if (( VERBOSE )); then
  xcodebuild \
    -project IsaacSwift.xcodeproj \
    -scheme IsaacSwift \
    -configuration Debug \
    -destination "generic/platform=iOS" \
    -sdk iphoneos \
    -derivedDataPath .derivedData \
    CODE_SIGNING_ALLOWED=NO \
    build
else
  echo "Building IsaacSwift (iphoneos, Debug)..."
  xcodebuild \
    -project IsaacSwift.xcodeproj \
    -scheme IsaacSwift \
    -configuration Debug \
    -destination "generic/platform=iOS" \
    -sdk iphoneos \
    -derivedDataPath .derivedData \
    CODE_SIGNING_ALLOWED=NO \
    build 2>&1 | grep -Ei 'error:|warning:|FAILED|SUCCEEDED'
fi
