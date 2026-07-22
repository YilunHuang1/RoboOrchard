#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
ROBO_ROOT=$(readlink -f "${SCRIPT_DIR}/../../..")
WORKSPACE_ROOT=$(dirname "${ROBO_ROOT}")
PYTHON=${AORTA_PYTHON:-"${WORKSPACE_ROOT}/docker_env/venv/robot-venv/bin/python"}
AORTA_RUNTIME=${AORTA_RUNTIME:-"${WORKSPACE_ROOT}/docker_env/aorta-runtime"}
REFERENCE_IMPL="${ROBO_ROOT}/docs/aorta-migration/reference-impl"

RUNTIME_DIR=${AORTA_RUNTIME_DIR:-"${XDG_RUNTIME_DIR:-/tmp}/robo-orchard-aorta-${UID}"}
SESSION_CONFIG="${RUNTIME_DIR}/desktop_session.json5"
SSH_HOST=${AORTA_SSH_HOST:-sh-106-s100}
LOCAL_ZENOH_PORT=${AORTA_LOCAL_ZENOH_PORT:-17447}
GROUP=${AORTA_GROUP:-robo_orchard_demo}

mkdir -p "${RUNTIME_DIR}"
chmod 700 "${RUNTIME_DIR}"

pid_is_running() {
    local pid_file=$1
    [[ -s "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" 2>/dev/null
}

cleanup_failed_start() {
    set +e
    for name in master muxer tunnel; do
        pid_file="${RUNTIME_DIR}/${name}.pid"
        if pid_is_running "${pid_file}"; then
            kill "$(cat "${pid_file}")" 2>/dev/null
        fi
        rm -f "${pid_file}"
    done
    rm -f "${SESSION_CONFIG}"
}

for name in tunnel muxer master; do
    if pid_is_running "${RUNTIME_DIR}/${name}.pid"; then
        echo "Aorta desktop stack is already running (${name})."
        echo "Stop it first: bash ${SCRIPT_DIR}/stop_aorta_master.sh"
        exit 1
    fi
    rm -f "${RUNTIME_DIR}/${name}.pid"
done

if (echo >/dev/tcp/127.0.0.1/"${LOCAL_ZENOH_PORT}") 2>/dev/null; then
    echo "Zenoh tunnel port is already in use: ${LOCAL_ZENOH_PORT}" >&2
    echo "Stop the stale tunnel before starting Aorta." >&2
    exit 1
fi

if [[ ! -x "${PYTHON}" ]]; then
    echo "Python environment not found: ${PYTHON}" >&2
    exit 1
fi
if [[ ! -d "${AORTA_RUNTIME}/aorta" ]]; then
    echo "Aorta runtime not found: ${AORTA_RUNTIME}" >&2
    exit 1
fi

export PYTHONPATH="${AORTA_RUNTIME}:${REFERENCE_IMPL}:${PYTHONPATH:-}"
"${PYTHON}" -c \
    "import aorta, piper_sdk, arm_joint_state_schema_meta, arm_trigger_schema_meta"

if ! ip link show can_left_mst >/dev/null 2>&1; then
    echo "Initializing can_left_mst..."
    bash "${SCRIPT_DIR}/rename-can.sh"
fi
if ! ip link show can_left_mst | grep -q "UP"; then
    echo "CAN interface can_left_mst is not UP." >&2
    exit 1
fi

trap cleanup_failed_start EXIT
trap 'exit 130' INT TERM

remote_config="${RUNTIME_DIR}/s100_session.json5.tmp"
scp -q "${SSH_HOST}:/app_param/zenoh/s100_session.json5" "${remote_config}"
"${PYTHON}" - "${remote_config}" "${SESSION_CONFIG}" "${LOCAL_ZENOH_PORT}" <<'PY'
import re
import sys

source, destination, port = sys.argv[1:]
text = open(source).read()
text, count = re.subn(
    r"tcp/127\.0\.0\.1:7447",
    f"tcp/127.0.0.1:{port}",
    text,
    count=1,
)
if count != 1:
    raise SystemExit("Could not locate the S100 Zenoh endpoint")
text = "\n".join(
    line for line in text.splitlines() if "dictionary_file:" not in line
) + "\n"
text = text.replace(
    "shared_memory: {\n      enabled: true",
    "shared_memory: {\n      enabled: false",
)
open(destination, "w").write(text)
PY
rm -f "${remote_config}"
chmod 600 "${SESSION_CONFIG}"

nohup ssh -N \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=15 \
    -L "127.0.0.1:${LOCAL_ZENOH_PORT}:127.0.0.1:7447" \
    "${SSH_HOST}" \
    </dev/null >"${RUNTIME_DIR}/tunnel.log" 2>&1 &
echo $! >"${RUNTIME_DIR}/tunnel.pid"

for _ in $(seq 1 50); do
    if pid_is_running "${RUNTIME_DIR}/tunnel.pid" \
        && (echo >/dev/tcp/127.0.0.1/"${LOCAL_ZENOH_PORT}") 2>/dev/null; then
        break
    fi
    sleep 0.1
done
if ! pid_is_running "${RUNTIME_DIR}/tunnel.pid" \
    || ! (echo >/dev/tcp/127.0.0.1/"${LOCAL_ZENOH_PORT}") 2>/dev/null; then
    echo "SSH tunnel failed; see ${RUNTIME_DIR}/tunnel.log" >&2
    exit 1
fi

export AORTA_GROUP="${GROUP}"
export ZENOH_SESSION_CONFIG_URI="${SESSION_CONFIG}"

nohup "${PYTHON}" "${REFERENCE_IMPL}/take_over_aorta.py" \
    --node-name takeover_muxer_node \
    --replay-time-s 0.0 \
    </dev/null >"${RUNTIME_DIR}/muxer.log" 2>&1 &
echo $! >"${RUNTIME_DIR}/muxer.pid"

nohup "${PYTHON}" "${REFERENCE_IMPL}/single_aorta.py" \
    --node-name robot_left_master_controller \
    --can-port can_left_mst \
    --auto-enable-arm-ctrl false \
    --enable-mit-ctrl false \
    --joint-state-topic aorta/master/joint_left \
    --status-topic aorta/master/status_left \
    --ee-pose-topic aorta/master/end_pose_left \
    --joint-cmd-topic aorta/robot/left/joint_cmd \
    --enable-service aorta/robot/left_master/enable_ctrl \
    --reset-service aorta/robot/left_master/reset_ctrl \
    </dev/null >"${RUNTIME_DIR}/master.log" 2>&1 &
echo $! >"${RUNTIME_DIR}/master.pid"

sleep 2
for name in tunnel muxer master; do
    if ! pid_is_running "${RUNTIME_DIR}/${name}.pid"; then
        echo "${name} failed; see ${RUNTIME_DIR}/${name}.log" >&2
        exit 1
    fi
done

trap - EXIT INT TERM
echo "Aorta desktop stack started in safe AUTONOMOUS mode."
echo "Logs: ${RUNTIME_DIR}"
echo "After aligning both arms, start linkage with:"
echo "  bash ${SCRIPT_DIR}/trigger_aorta_takeover.sh"
