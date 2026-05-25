#!/bin/bash
# Stage only the small, important flight MCAP files for git backup.
#
# Usage:
#   ./dev_logs/stage_flight_backups.sh [flight_root] [max_size]
#
# Examples:
#   ./dev_logs/stage_flight_backups.sh
#   ./dev_logs/stage_flight_backups.sh /home/ws/dev_logs/flights 100M

set -euo pipefail

ROOT_DIR="${1:-/home/ws/dev_logs/flights}"
MAX_SIZE="${2:-100M}"

if [[ ! -d "$ROOT_DIR" ]]; then
  echo "Flight root not found: $ROOT_DIR" >&2
  exit 1
fi

cd /home/ws

mapfile -d '' files < <(
  {
    find "$ROOT_DIR" -type f -name '*_0.mcap' -size -"$MAX_SIZE" -print0
    find "$ROOT_DIR" -type f -name '*-pass*.mcap' -size -"$MAX_SIZE" -print0
  }
)

if (( ${#files[@]} == 0 )); then
  echo "No flight MCAP files under $MAX_SIZE found in $ROOT_DIR"
  exit 0
fi

git add "${files[@]}"

echo "Staged ${#files[@]} flight MCAP file(s) under $MAX_SIZE."