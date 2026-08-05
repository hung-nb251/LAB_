cmake_minimum_required(VERSION 3.8)
project(motoman_hc10dtp_support)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
