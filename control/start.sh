#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
for a in "$@"; do
  if [[ "$a" == '--dry-run' || "$a" == '--help' || "$a" == '-h' ]]; then exec python3 "$ROOT/scripts/remote_session.py" "$@"; fi
done
[[ -x "$ROOT/program/build/remote_bridge" && -f "$ROOT/program/build/liblift_profile.so" ]] || "$ROOT/build.sh"
# Interface setup sends no enable/motion frames. Existing matching CAN is reused.
"$ROOT/program/setup_can.sh" can0
mkdir -p "$ROOT/logs"
set -o pipefail
python3 -u "$ROOT/scripts/remote_session.py" "$@" 2>&1 | (trap '' HUP; exec tee -i "$ROOT/logs/console_$(date +%Y%m%d_%H%M%S)_$$.log")
