cmake_minimum_required(VERSION 3.5)
project(capra_landolt_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 1. Abhängigkeiten suchen
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(capra_landolt_msgs REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# 2. Haupt-Knoten (Executable) erstellen und konfigurieren
add_executable(landolt_node
  src/LandoltNode.cpp
)

ament_target_dependencies(landolt_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  capra_landolt_msgs
)

# Das Linken der Haupt-Node gehört DIREKT hierhin, vor die Tests/Installationen!
target_link_libraries(landolt_node ${OpenCV_LIBRARIES})


# 3. Tests (falls aktiviert)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # C++ Test-Executable erstellen
  add_executable(landolt_test_exe test/landolt_test.cpp)
  
  target_link_libraries(landolt_test_exe 
    ${OpenCV_LIBRARIES}
    gtest
    gtest_main
  )
  
  ament_target_dependencies(landolt_test_exe
    rclcpp
    cv_bridge
    image_transport
    capra_landolt_msgs
    sensor_msgs
  )

  # Den Integrationstest registrieren
  add_launch_test(launch/test_landolt.launch.py)
endif()


# 4. Installations-Regeln (Erst ausführen, wenn alle Targets fertig konfiguriert sind)
install(TARGETS landolt_node landolt_test_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Sicherstellen, dass auch der Test-Ordner (mit den Bildern) im Share-Pfad landet!
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/
)

# 5. Paket-Makro abschließen
ament_package()