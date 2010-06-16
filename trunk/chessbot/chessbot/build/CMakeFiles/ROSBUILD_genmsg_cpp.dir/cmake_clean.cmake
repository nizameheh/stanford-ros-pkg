FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg/cpp/chessbot/ChessbotAction.h"
  "../msg/cpp/chessbot/ChessbotGoal.h"
  "../msg/cpp/chessbot/ChessbotActionGoal.h"
  "../msg/cpp/chessbot/ChessbotResult.h"
  "../msg/cpp/chessbot/ChessbotActionResult.h"
  "../msg/cpp/chessbot/ChessbotFeedback.h"
  "../msg/cpp/chessbot/ChessbotActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
