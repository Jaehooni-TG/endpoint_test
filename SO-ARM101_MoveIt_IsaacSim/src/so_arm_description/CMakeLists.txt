cmake_minimum_required(VERSION 3.8)
project(so_arm_description)

find_package(ament_cmake REQUIRED)

# Install robot description assets
install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES joints_properties.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

