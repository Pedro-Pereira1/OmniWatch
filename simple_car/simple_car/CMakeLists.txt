cmake_minimum_required(VERSION 3.5)
project(simple_car)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Instalar o ficheiro Python
install(
  PROGRAMS
  src/car_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar os ficheiros de launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar os modelos SDF
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
