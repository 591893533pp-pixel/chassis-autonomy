#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -L)
source "$HERE/../scripts/env.sh"
exec python3 "$HERE/run_mapping.py" "$@"
