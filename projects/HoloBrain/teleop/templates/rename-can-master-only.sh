#!/bin/bash
declare -A USB_PORTS

# PC side distributed mode: only master arm CAN
USB_PORTS["1-2:1.0"]="can_left_mst:1000000"

# Whether to ignore CAN quantity check (default false)
IGNORE_CHECK=false

# NOTE: This is a template for distributed mode where only the master arm
# is connected to the PC. The slave arm CAN is on the robot dog (S100).
# Copy this to teleop/rename-can.sh and modify USB_PORTS to match your hardware.
#
# To find your USB port, run: bash teleop/find-all-can-port.sh
