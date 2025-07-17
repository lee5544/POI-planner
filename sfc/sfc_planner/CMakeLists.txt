cmake_minimum_required(VERSION 3.0.2)
project(sfc_planner)

ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)

# # Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sfc_env

  nlopt # ubuntu18.04 需要指定该名称

)

find_package(NLopt REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sfc_planner
  CATKIN_DEPENDS sfc_env

  # DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(NLopt REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(sfc_planner
  include/HybirdAstar.h
  include/SFCNlopt.h
  include/UniformBspline.h
  src/HybirdAstar.cpp
  src/SFCNlopt.cpp
  src/UniformBspline.cpp
)

target_link_libraries(sfc_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${OpenCV_LIBS}
  ${NLOPT_LIBRARIES}
)

# add_executable(test_sfcsearch
#   include/HybirdAstar.h
#   include/AstarSearch.h
#   include/JumpPointSearch.h
#   include/SFCNlopt.h

#   src/HybirdAstar.cpp
#   src/AstarSearch.cpp
#   src/JumpPointSearch.cpp
#   src/SFCNlopt.cpp

#   test/test_sfcsearch.cpp
# )

# target_link_libraries(test_sfcsearch
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )



add_executable(test_sfc
  include/HybirdAstar.h
  include/AstarSearch.h
  include/JumpPointSearch.h
  include/SFCNlopt.h
  include/PathNlopt.h

  src/HybirdAstar.cpp
  src/AstarSearch.cpp
  src/JumpPointSearch.cpp
  src/SFCNlopt.cpp
  src/PathNlopt.cpp

  test/test_sfc.cpp
)

target_link_libraries(test_sfc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(test_search
  include/HybirdAstar.h
  include/AstarSearch.h
  include/JumpPointSearch.h
  include/SFCNlopt.h
  include/PathNlopt.h

  src/HybirdAstar.cpp
  src/AstarSearch.cpp
  src/JumpPointSearch.cpp
  src/SFCNlopt.cpp
  src/PathNlopt.cpp

  test/test_search.cpp
)

target_link_libraries(test_search
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(test_poi
  include/HybirdAstar.h
  include/AstarSearch.h
  include/JumpPointSearch.h
  include/SFCNlopt.h
  include/PathNlopt.h

  src/HybirdAstar.cpp
  src/AstarSearch.cpp
  src/JumpPointSearch.cpp
  src/SFCNlopt.cpp
  src/PathNlopt.cpp

  test/test_poi.cpp
)

target_link_libraries(test_poi
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(test_2d
  include/HybirdAstar.h
  include/AstarSearch.h
  include/JumpPointSearch.h
  include/SFCNlopt.h
  include/PathNlopt.h

  src/HybirdAstar.cpp
  src/AstarSearch.cpp
  src/JumpPointSearch.cpp
  src/SFCNlopt.cpp
  src/PathNlopt.cpp

  test/test_2d.cpp
)

target_link_libraries(test_2d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
