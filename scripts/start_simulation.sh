#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-${HOME}/isaacsim}"

echo "Starting Isaac Sim..."

if [[ ! -x "$ISAAC_SIM_DIR/isaac-sim.sh" ]]; then
    echo "[ERROR] Isaac Sim launcher not found: $ISAAC_SIM_DIR/isaac-sim.sh" >&2
    echo "        Set ISAAC_SIM_DIR to the local Isaac Sim installation." >&2
    exit 1
fi

cd "$ISAAC_SIM_DIR"
exec ./isaac-sim.sh
