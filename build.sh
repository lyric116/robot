#!/usr/bin/env bash
set -euo pipefail
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PACKAGE_DIR"
cmake -S . -B "build-$(uname -m)" -DCMAKE_BUILD_TYPE=Release
cmake --build "build-$(uname -m)" -j "${BUILD_JOBS:-2}"
ctest --test-dir "build-$(uname -m)" --output-on-failure
