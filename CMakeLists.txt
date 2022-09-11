cmake_minimum_required(VERSION 3.14)
project(local_pose_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic)

add_definitions(${GeographicLib_DEFINITIONS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/local_pose_publisher.cpp
  src/lanelet2_utils.cpp)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${GeographicLib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${BOOST_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  Boost
  Eigen3)

target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${BOOST_LIBRARIES})

ament_auto_add_executable(${PROJECT_NAME}_exe src/local_pose_publisher.cpp)
add_dependencies(${PROJECT_NAME}_exe ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_exe
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch)