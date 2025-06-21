cmake_minimum_required(VERSION 3.5)
project(simple_car)

# 🔧 C++ settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 🔍 Encontra pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

# 📁 Incluir diretórios
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# 🧩 Compilar plugin C++
add_library(color_changer_plugin SHARED
  src/color_changer_plugin.cpp
)

ament_target_dependencies(color_changer_plugin
  rclcpp
  std_msgs
)

target_link_libraries(color_changer_plugin
  ${GAZEBO_LIBRARIES}
)

# 📦 Instalação do plugin
install(
  TARGETS color_changer_plugin
  LIBRARY DESTINATION lib
)

# 🐍 Instalar os nós Python
install(
  PROGRAMS
  src/car_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_follower
)

# 🚀 Instalar ficheiros de launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 🧱 Instalar modelos SDF
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

ament_package()
