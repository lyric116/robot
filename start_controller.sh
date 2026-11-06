#!/usr/bin/env bash
set -euo pipefail
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PACKAGE_DIR"
python3 scripts/verify_package.py
CPU_ARCH="$(uname -m)"
if [[ "${1:-}" == "--self-test" ]]; then
  exec "$PACKAGE_DIR/bin/$CPU_ARCH/throw_self_test" "$PACKAGE_DIR"
fi
if [[ ! -x "$PACKAGE_DIR/bin/$CPU_ARCH/g1_throw" ]]; then
  echo "No native binary for $CPU_ARCH yet. Run: bash build.sh" >&2
  exit 2
fi
# Default mode is read-only. --control enables the body command publisher;
# motion still requires the two explicit remote-control button steps.
exec "$PACKAGE_DIR/bin/$CPU_ARCH/g1_throw" --root "$PACKAGE_DIR" --network "${G1_NETWORK:-eth0}" "$@"
