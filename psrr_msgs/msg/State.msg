# This message represents the n-dimensional state of the Self-Reconfigurable Robots in world frame
# pose, twist and accel is from the base frame of the robot
# joint_state represents each reconfigurable gaits of the robot

std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel
sensor_msgs/JointState joint_state