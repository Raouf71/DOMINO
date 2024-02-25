cmake_minimum_required(VERSION 3.0.2)
project(update_map)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  octomap_ros
  roscpp
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES update_map
CATKIN_DEPENDS geometry_msgs nav_msgs octomap_ros roscpp std_msgs tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(update_map_node
  src/update_map.cpp)

target_link_libraries(update_map_node
  ${catkin_LIBRARIES}
)

# add_executable(view_map_node
#   src/view_map.cpp)

# target_link_libraries(view_map_node
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
# )

add_executable(create_2d_octomap_node 
  src/create_2d_octomap.cpp)

target_link_libraries(create_2d_octomap_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)


