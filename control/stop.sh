#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
exec python3 "$ROOT/scripts/owned_process.py" 
