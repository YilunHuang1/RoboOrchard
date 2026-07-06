set -ex

# Slave arm only: for model inference without master arm
ros2 launch robo_orchard_teleop_ros2 piper_dagger_infer.launch.py \
    replay_time_s:=0.0
