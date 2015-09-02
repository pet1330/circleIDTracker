cmake_minimum_required(VERSION 2.8.3)
project(circle_detection)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    image_transport
    message_generation
    mongodb_store
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
)

find_package(Boost REQUIRED)

add_message_files(
   FILES
   detection_results.msg
   detection_results_array.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include CATKIN_DEPENDS geometry_msgs image_transport message_runtime mongodb_store sensor_msgs tf visualization_msgs
)
include_directories(${GSL_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Libraries
add_library(CCircleDetect src/CCircleDetect.cpp)
add_library(CRawImage src/CRawImage.cpp)
add_library(CTransformation src/CTransformation.cpp)
add_library(FindCircle src/FindCircle.cpp)

add_dependencies(CTransformation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(FindCircle ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(CTransformation
${GSL_LIBRARIES}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)
 
# Executables

add_executable(circle_finder src/FindCircle.cpp)
add_dependencies(circle_finder ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(circle_finder
    CCircleDetect
    CRawImage
    CTransformation
    gsl
    blas
    ${catkin_LIBRARIES}
)

install(TARGETS
    CCircleDetect
    CRawImage
    CTransformation
    circle_finder ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY doc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GSL REQUIRED gsl)
link_directories(${GSL_LIBRARY_DIRS})
