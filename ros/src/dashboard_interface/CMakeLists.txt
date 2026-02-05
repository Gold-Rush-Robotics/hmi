cmake_minimum_required(VERSION 3.8)
project(dashboard_interface)

# Dependencies.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface. 
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dashboard.msg"
)

# Export.
ament_package()
