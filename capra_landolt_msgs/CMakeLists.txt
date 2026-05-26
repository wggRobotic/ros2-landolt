cmake_minimum_required(VERSION 3.5)
project(capra_landolt_msgs)

# 1. Compiler-Standard setzen (wichtig für ROS2 C++ Support)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Pakete finden
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 3. Interfaces generieren
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Landolts.msg"
  "msg/BoundingCircles.msg"
  "msg/Point2f.msg"
  DEPENDENCIES std_msgs
)

# 4. WICHTIG: Die Interfaces exportieren, damit andere Pakete sie nutzen können
ament_export_dependencies(rosidl_default_runtime)

# 5. Paket registrieren
ament_package()