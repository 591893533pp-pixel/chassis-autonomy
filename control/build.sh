#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
# This repository has one canonical control source; no legacy duplicate synchronization.
mkdir -p "$ROOT/logs"
set -o pipefail
{ cmake -S "$ROOT/program" -B "$ROOT/program/build" -DCMAKE_BUILD_TYPE=Release; cmake --build "$ROOT/program/build" -j 4; ctest --test-dir "$ROOT/program/build" --output-on-failure; } 2>&1 | tee "$ROOT/logs/build_$(date +%Y%m%d_%H%M%S).log"
