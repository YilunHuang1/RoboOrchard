set -ex

# PC side: launch master arm controller + takeover muxer only
ros2 launch robo_orchard_teleop_ros2 piper_dagger_master.launch.py \
    replay_time_s:=0.0
