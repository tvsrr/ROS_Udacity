cmake_minimum_required(VERSION 2.8.3)
project(helper_codes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(unpause_physics src/unpause_physics.cpp)
target_link_libraries(unpause_physics ${catkin_LIBRARIES})
