
#header mainly for timestamping purpose

std_msgs/Header header

# 0 : stop
# 1 : StraigthPath (without planification, no basic obstacle stop yet)
# 2 : PurePursuit (use astar planification and obstacle avoidance)
# 3 : WallFollower (without planification, no basic obstacle stop yet)
# 4 : WallStop (without planification, no basic obstacle stop yet)
int32 navigation_type

#goal_pose
geometry_msgs/Pose pose