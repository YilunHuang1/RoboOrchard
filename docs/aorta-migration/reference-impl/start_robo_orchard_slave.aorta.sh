#!/bin/bash
# Aorta-only S100 slave-arm launcher (migration PR-B draft).
#
# Reference version of vita-robot's
#   script/s100/systemd/start_robo_orchard_slave.sh
# Replaces the ROS2 `--ros-args -r ... :=/puppet/...` remaps + AMENT/ROS env
# with Aorta env + explicit topic args. CAN + piper_sdk persistence unchanged.
# This file lands in the vita-robot PR, not RoboOrchard.

set -euo pipefail

ROOT_PATH=/app
APP_ROOT=${ROOT_PATH}/robo_orchard
ENV_FILE=/app_param/robo_orchard/robo_orchard.env
PERSISTENT_PYTHON_DIR=/app_param/robo_orchard/python

# Load the same operator configuration for foreground and systemd starts.
if [[ -r "${ENV_FILE}" ]]; then
    set -a
    source "${ENV_FILE}"
    set +a
fi

CAN_PORT=${ROBO_ORCHARD_CAN_PORT:-can0}
AUTO_ENABLE_ARM_CTRL=${ROBO_ORCHARD_AUTO_ENABLE_ARM_CTRL:-false}

# --- Aorta runtime configuration ---------------------------------------------
# libaorta_core.so location on the S100 image (the SDK loads it via ctypes).
export AORTA_CORE_FFI_LIB=${AORTA_CORE_FFI_LIB:-/app/aorta/lib/libaorta_core.so}
# Group MUST match the desktop master process (single_aorta / muxer) so the
# cross-host aorta/** topics rendezvous. Default to the robot SN if set.
export AORTA_GROUP=${AORTA_GROUP:-${ROBOT_SN:-robo_orchard_demo}}
# Aorta rides the existing S100 zenoh session config (client mode). Uncomment /
# override if this process needs a specific session:
# export ZENOH_SESSION_CONFIG_URI=/app_param/zenoh/s100_session.json5

source ${ROOT_PATH}/script/env.sh

if ! ip link show "${CAN_PORT}" >/dev/null 2>&1; then
    echo "[robo_orchard] CAN interface not found: ${CAN_PORT}" >&2
    exit 1
fi

# PYTHONPATH: the piper package (now containing single_aorta + arm_bridge), the
# Aorta Python SDK, and the generated *_schema_meta modules. AMENT/ROS msg paths
# are gone.
export PYTHONPATH="${PERSISTENT_PYTHON_DIR}:${APP_ROOT}/workspace/ros2_ws/src/robo_orchard_piper_ros2:${APP_ROOT}/aorta/python:${APP_ROOT}/aorta/schema_meta:${PYTHONPATH:-}"

if ! python3 -c "import piper_sdk" >/dev/null 2>&1; then
    # The root overlay is reset at reboot; keep the Piper SDK on /app_param.
    mkdir -p "${PERSISTENT_PYTHON_DIR}"
    echo "[robo_orchard] Installing piper_sdk into ${PERSISTENT_PYTHON_DIR}" >&2
    python3 -m pip install --disable-pip-version-check --no-cache-dir --upgrade \
        --target "${PERSISTENT_PYTHON_DIR}" piper_sdk==0.4.1
fi

python3 -c "import piper_sdk, numpy, scipy, aorta" || {
    echo "[robo_orchard] Missing S100 Python dependency (piper_sdk/numpy/scipy/aorta)" >&2
    exit 1
}

exec python3 -m robo_orchard_piper_ros2.single_aorta \
    --node-name robot_left_controller \
    --can-port "${CAN_PORT}" \
    --auto-enable-arm-ctrl "${AUTO_ENABLE_ARM_CTRL}" \
    --enable-mit-ctrl false \
    --joint-cmd-topic   aorta/robot/left/joint_cmd \
    --joint-state-topic aorta/puppet/joint_left \
    --status-topic      aorta/puppet/status_left \
    --ee-pose-topic     aorta/puppet/end_pose_left \
    --enable-service    aorta/robot/left/enable_ctrl \
    --reset-service     aorta/robot/left/reset_ctrl
