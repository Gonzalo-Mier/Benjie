include_directories(.)

file(GLOB hdrs "*.h*")
file(GLOB srcs "*.c*")

set(aruco_hdr ${hdrs})
set(aruco_src ${srcs})

rosbuild_add_library( aruco ${aruco_src} ${aruco_hdr})

