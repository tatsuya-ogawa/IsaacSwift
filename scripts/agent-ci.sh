#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/.."

echo "=== Starting Agent CI (Quiet Mode) ==="

./scripts/build-device.sh "$@"
echo ""
./scripts/test-smoke.sh "$@"

echo "=== Agent CI Complete ==="
