# TaskCommand.msg
uint8 command_type
uint8 GRASP=1
uint8 DELIVER=2
uint8 RESET=3
geometry_msgs/PoseStamped target_pose
