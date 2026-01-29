cmake_minimum_required(VERSION 3.8)
project(hmi_com_interface)

# Dependencies.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface. 
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NodeInfoSrv.srv"
)

# Export.
ament_package()
