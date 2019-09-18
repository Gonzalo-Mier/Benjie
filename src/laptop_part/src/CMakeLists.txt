add_subdirectory( aruco ) 

include_directories (. ./aruco)

set(LIBRARIES aruco opencv_core opencv_highgui opencv_calib3d)

rosbuild_add_executable(kinect_aruco kinect_aruco.cpp)
target_link_libraries(kinect_aruco ${LIBRARIES})


