FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/benjie/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/benjie/msg/__init__.py"
  "../src/benjie/msg/_Marcador.py"
  "../src/benjie/msg/_MarcadorArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
