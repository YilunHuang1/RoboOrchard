#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
ROBO_ROOT=$(readlink -f "${SCRIPT_DIR}/../../..")
WORKSPACE_ROOT=$(dirname "${ROBO_ROOT}")
PYTHON=${AORTA_PYTHON:-"${WORKSPACE_ROOT}/docker_env/venv/robot-venv/bin/python"}
AORTA_RUNTIME=${AORTA_RUNTIME:-"${WORKSPACE_ROOT}/docker_env/aorta-runtime"}
RUNTIME_DIR=${AORTA_RUNTIME_DIR:-"${XDG_RUNTIME_DIR:-/tmp}/robo-orchard-aorta-${UID}"}
SESSION_CONFIG="${RUNTIME_DIR}/desktop_session.json5"

pid_is_running() {
    local pid_file=$1
    [[ -s "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" 2>/dev/null
}

stop_pid() {
    local name=$1
    local pid_file="${RUNTIME_DIR}/${name}.pid"
    if ! pid_is_running "${pid_file}"; then
        rm -f "${pid_file}"
        return
    fi

    local pid
    pid=$(cat "${pid_file}")
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
        if ! kill -0 "${pid}" 2>/dev/null; then
            break
        fi
        sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
        kill -9 "${pid}" 2>/dev/null || true
    fi
    rm -f "${pid_file}"
}

if [[ -r "${SESSION_CONFIG}" ]] && pid_is_running "${RUNTIME_DIR}/muxer.pid"; then
    export PYTHONPATH="${AORTA_RUNTIME}:${PYTHONPATH:-}"
    export AORTA_GROUP=${AORTA_GROUP:-robo_orchard_demo}
    export ZENOH_SESSION_CONFIG_URI="${SESSION_CONFIG}"
    timeout 8 "${PYTHON}" "${SCRIPT_DIR}/aorta_control.py" stop \
        >"${RUNTIME_DIR}/stop.log" 2>&1 || {
        echo "Warning: STOP RPC failed; terminating local publishers anyway." >&2
    }
fi

stop_pid master
stop_pid muxer
stop_pid tunnel
rm -f "${SESSION_CONFIG}" "${RUNTIME_DIR}/s100_session.json5.tmp"

echo "Aorta desktop stack stopped."
echo "The S100 slave service is left running and will receive no desktop commands."
