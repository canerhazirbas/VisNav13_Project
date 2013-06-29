FILE(REMOVE_RECURSE
  "msg_gen"
  "src/visnav_project/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "cfg/cpp/visnav_project/PidParameterConfig.h"
  "docs/PidParameterConfig.dox"
  "docs/PidParameterConfig-usage.dox"
  "src/visnav_project/cfg/PidParameterConfig.py"
  "docs/PidParameterConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
