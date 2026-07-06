set -ex

# Robot dog (S100) side: launch slave arm controller only
# Make sure ROS_DOMAIN_ID matches the PC side
ros2 launch robo_orchard_teleop_ros2 piper_dagger_slave.launch.py
