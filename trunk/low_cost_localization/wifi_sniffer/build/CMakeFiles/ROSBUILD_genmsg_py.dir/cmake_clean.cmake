FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/wifi_sniffer/msg/__init__.py"
  "../src/wifi_sniffer/msg/_WifiScan.py"
  "../src/wifi_sniffer/msg/_WifiSniff.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
