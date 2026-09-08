cmake_minimum_required(VERSION 3.8)
project(drdo_gz_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  launch
  px4_airframes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
