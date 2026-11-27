#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -L)
exec python3 "$HERE/tools/stage_source.py" "$@"
