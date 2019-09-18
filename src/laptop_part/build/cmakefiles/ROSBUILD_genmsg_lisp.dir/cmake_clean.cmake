FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/benjie/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Marcador.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Marcador.lisp"
  "../msg_gen/lisp/MarcadorArray.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MarcadorArray.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
