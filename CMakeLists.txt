cmake_minimum_required(VERSION 3.8)
project(waam_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)


#--- Include
include_directories(
  include
  $ENV{HOME}/baumer_neoapi_sdk/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  $ENV{HOME}/baumer_neoapi_sdk/lib
)

#--- Executable: node_validate_lidar_camera_calib
add_executable(node_baumer_ir_cam
  src/node_baumer_ir_cam.cpp
)
ament_target_dependencies(node_baumer_ir_cam
  # rclcpp 
  # std_msgs 
  # sensor_msgs 
)
target_link_libraries(node_baumer_ir_cam
  neoapi_cpp
  ${OpenCV_LIBS}
  # cv_bridge::cv_bridge
  # image_transport::image_transport
)
install(TARGETS
	node_baumer_ir_cam
	DESTINATION lib/${PROJECT_NAME}
)


# #--- Executable: node_mask2former.py
# install(PROGRAMS 
# 		src/node_mask2former.py 
# 		src/Mask2Former.py
# 		DESTINATION lib/${PROJECT_NAME})


#--- Install folder
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY config launch rviz tmux DESTINATION share/${PROJECT_NAME})


# Find Python modules in the 'src' folder
ament_python_install_package(src)

ament_package()
