#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
ROBO_ROOT=$(readlink -f "${SCRIPT_DIR}/../../..")
WORKSPACE_ROOT=$(dirname "${ROBO_ROOT}")
PYTHON=${AORTA_PYTHON:-"${WORKSPACE_ROOT}/docker_env/venv/robot-venv/bin/python"}
AORTA_RUNTIME=${AORTA_RUNTIME:-"${WORKSPACE_ROOT}/docker_env/aorta-runtime"}
RUNTIME_DIR=${AORTA_RUNTIME_DIR:-"${XDG_RUNTIME_DIR:-/tmp}/robo-orchard-aorta-${UID}"}

for name in tunnel muxer master; do
    pid_file="${RUNTIME_DIR}/${name}.pid"
    if [[ ! -s "${pid_file}" ]] \
        || ! kill -0 "$(cat "${pid_file}")" 2>/dev/null; then
        echo "Aorta ${name} process is not running." >&2
        echo "Run start_aorta_master.sh successfully before takeover." >&2
        exit 1
    fi
done

if [[ ! -r "${RUNTIME_DIR}/desktop_session.json5" ]]; then
    echo "Aorta desktop stack is not configured. Run start_aorta_master.sh first." >&2
    exit 1
fi

export PYTHONPATH="${AORTA_RUNTIME}:${PYTHONPATH:-}"
export AORTA_GROUP=${AORTA_GROUP:-robo_orchard_demo}
export ZENOH_SESSION_CONFIG_URI="${RUNTIME_DIR}/desktop_session.json5"

exec "${PYTHON}" "${SCRIPT_DIR}/aorta_control.py" takeover "$@"
