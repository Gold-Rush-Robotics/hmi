cmake_minimum_required(VERSION 3.8)
project(hmi_com_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface. 
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NodeInfoSrv.srv"
)

# Testing and linting. 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export.
ament_package()
