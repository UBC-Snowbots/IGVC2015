#Local map: Simple mapping node publishes this and simple ai node subscribes to this
#New Message format
std_msgs/Header header
geometry_msgs/Pose2D pose
nav_msgs/OccupancyGrid map


# (int8[] map, uint8 x, uint8 y, uint8 rotation)

#bool[] map     #local map
#uint8 x	       #x-coordinate (0 < x < 1)
#uint8 y	       #y-coordinate (0 < y < 1)
#uint8 rotation #rotation angle (North=0,East=1,South=2,West=3)

# Message that Justin wrote up earlier
#std_msgs/Header header
#uint8[] map
#geometry_msgs/Pose2D
