FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/CartesianArmServerAction.msg"
  "../msg/CartesianArmServerGoal.msg"
  "../msg/CartesianArmServerActionGoal.msg"
  "../msg/CartesianArmServerResult.msg"
  "../msg/CartesianArmServerActionResult.msg"
  "../msg/CartesianArmServerFeedback.msg"
  "../msg/CartesianArmServerActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
