cmake_minimum_required(VERSION 3.8)
project(vision_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPickedObject.srv"
)

install(PROGRAMS
  nodes/dino_any6d_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dino_any6d_node
)

install(PROGRAMS
  nodes/dino_all_object_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dino_all_object_node
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
