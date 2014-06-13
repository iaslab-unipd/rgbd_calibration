cmake_minimum_required(VERSION 2.8.3)
project(rgbd_calibration)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp calibration_common geometry_msgs kinect
                                        eigen_conversions camera_info_manager cv_bridge pcl_ros image_transport)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  CheckerboardMsg.msg
#  CheckerboardArray.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interactive_checkerboard_finder rgbd_calibration
  CATKIN_DEPENDS roscpp calibration_common geometry_msgs kinect
                 eigen_conversions camera_info_manager cv_bridge pcl_ros image_transport
  DEPENDS eigen pcl opencv2
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a cpp library

add_library(rgbd_calibration STATIC
  src/rgbd_calibration/calibration_node.cpp              include/rgbd_calibration/calibration_node.h
  src/rgbd_calibration/calibration.cpp                   include/rgbd_calibration/calibration.h
                                                         include/rgbd_calibration/globals.h
  src/rgbd_calibration/depth_undistortion_estimation.cpp include/rgbd_calibration/depth_undistortion_estimation.h
  src/rgbd_calibration/checkerboard_views.cpp            include/rgbd_calibration/checkerboard_views.h
  src/rgbd_calibration/checkerboard_views_extractor.cpp  include/rgbd_calibration/checkerboard_views_extractor.h
  src/rgbd_calibration/publisher.cpp                     include/rgbd_calibration/publisher.h
)

add_executable(rgbd_offline_calibration
  src/rgbd_calibration/offline_calibration_node.cpp      include/rgbd_calibration/offline_calibration_node.h
)

add_executable(simulation
  src/rgbd_calibration/simulation_node.cpp               include/rgbd_calibration/simulation_node.h
)

add_executable(test_calibration
  src/rgbd_calibration/test_node.cpp                     include/rgbd_calibration/test_node.h
  src/rgbd_calibration/calibration_test.cpp              include/rgbd_calibration/calibration_test.h
)

add_executable(data_collection
  src/rgbd_calibration/data_collection_node.cpp
)

## Add dependencies to the executable
# add_dependencies(calibration_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against

target_link_libraries(rgbd_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(rgbd_offline_calibration
  rgbd_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(simulation
  rgbd_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(test_calibration
  rgbd_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(data_collection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS calibration calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/polynomial_undistortion_matrix_multifit_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test   
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${CERES_LIBRARIES})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
