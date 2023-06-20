cmake_minimum_required(VERSION 3.5)
project(multi_radar_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Targets
ament_auto_add_library(multi_radar_fusion SHARED
  src/multi_radar_fusion.cpp
)

ament_auto_add_executable(multi_radar_fusion_node
  src/multi_radar_fusion_node.cpp
)

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
