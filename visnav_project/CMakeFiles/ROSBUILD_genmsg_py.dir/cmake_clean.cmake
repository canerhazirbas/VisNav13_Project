FILE(REMOVE_RECURSE
  "msg_gen"
  "src/visnav_project/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/visnav_project/msg/__init__.py"
  "src/visnav_project/msg/_State.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
