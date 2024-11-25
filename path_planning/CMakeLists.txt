cmake_minimum_required(VERSION 3.0)
project(path_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cmake_modules
    std_msgs
)
catkin_package()
find_package(Eigen3 REQUIRED)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(path_node
src/utils/sdqp.hpp
src/utils/astar.hpp
src/utils/lbfgs.hpp
src/utils/dijkstra.hpp
src/utils/spline.hpp
src/obstacle.hpp 
src/map.hpp
src/smooth.hpp
src/topp.hpp
src/mpc.hpp
src/path_node.cpp
)

add_executable(test_demo
src/utils/sdqp.hpp
src/obstacle.hpp
src/map.hpp
src/test.cpp
)

catkin_install_python(PROGRAMS scripts/plot_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(path_node ${catkin_LIBRARIES})