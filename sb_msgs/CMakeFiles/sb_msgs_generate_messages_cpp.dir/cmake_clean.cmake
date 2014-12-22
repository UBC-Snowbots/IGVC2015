FILE(REMOVE_RECURSE
  "CMakeFiles/sb_msgs_generate_messages_cpp"
  "../../devel/include/sb_msgs/jausout.h"
  "../../devel/include/sb_msgs/RobotState.h"
  "../../devel/include/sb_msgs/ServoCommand.h"
  "../../devel/include/sb_msgs/Velocity.h"
  "../../devel/include/sb_msgs/Pose.h"
  "../../devel/include/sb_msgs/IMU.h"
  "../../devel/include/sb_msgs/TurretCommand.h"
  "../../devel/include/sb_msgs/jausin.h"
  "../../devel/include/sb_msgs/LidarNav.h"
  "../../devel/include/sb_msgs/CarCommand.h"
  "../../devel/include/sb_msgs/VisionNav.h"
  "../../devel/include/sb_msgs/gps.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/sb_msgs_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
