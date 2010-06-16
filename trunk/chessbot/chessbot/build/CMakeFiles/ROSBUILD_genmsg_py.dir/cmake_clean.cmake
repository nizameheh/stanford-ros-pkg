FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/chessbot/msg/__init__.py"
  "../src/chessbot/msg/_ChessbotAction.py"
  "../src/chessbot/msg/_ChessbotGoal.py"
  "../src/chessbot/msg/_ChessbotActionGoal.py"
  "../src/chessbot/msg/_ChessbotResult.py"
  "../src/chessbot/msg/_ChessbotActionResult.py"
  "../src/chessbot/msg/_ChessbotFeedback.py"
  "../src/chessbot/msg/_ChessbotActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
