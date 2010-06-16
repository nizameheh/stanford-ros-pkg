FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/ChessbotAction.msg"
  "../msg/ChessbotGoal.msg"
  "../msg/ChessbotActionGoal.msg"
  "../msg/ChessbotResult.msg"
  "../msg/ChessbotActionResult.msg"
  "../msg/ChessbotFeedback.msg"
  "../msg/ChessbotActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
