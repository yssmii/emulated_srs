cmake_minimum_required(VERSION 3.0.2)

project(emulated_srs)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -std=c++11")

set(CMAKE_VERBOSE_MAKEFILE 1)

option(WITH_YOLO "YOLO" OFF)
option(WITH_YOLOV4 "YOLOv4" OFF)
if(WITH_YOLOV4)
  set(WITH_YOLO ON)
endif(WITH_YOLOV4)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  # actionlib
  # actionlib_msgs
  cv_bridge
  image_transport
  geometry_msgs
  message_generation
  )

find_package(OpenCV REQUIRED)

#set(esrs_LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/src/libeSRS)
#set(esrs_INCDIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
#set(esrs_LIBRARIES ${esrs_LIBDIR}/libsrss.a)
#set(esrs_LIBRARIES -lsrss)
#set(esrs_INCDIRS)

#find_package(OpenCV 2 REQUIRED)
#find_package(OpenCV 3 REQUIRED)
#if(NOT OpenCV_FOUND)
#  message(FATAL_ERROR "\n\n OpenCV 2 is missing\n\n")
#endif()
#set(CMAKE_REQUIRED_INCLUDES
#  ${CMAKE_REQUIRED_INCLUDES} ${OpenCV_INCLUDE_DIRS})
#set(esrs_LIBRARIES ${esrs_LIBRARIES} ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})
message( STATUS "DEBUG: OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS} )

#set(YOLO_LIBRARIES)

if(WITH_YOLO)
  find_package(CUDA 10 REQUIRED)
  if(NOT CUDA_FOUND)
    message(FATAL_ERROR "\n\n CUDA 10 is missing\n\n")
  endif()
  set(CUDA_LIBRARY_DIR ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

  if(DEFINED ENV{PATH_DARKNET})
    set(PATH_YOLO_DIR $ENV{PATH_DARKNET})
  else()
    set(PATH_YOLO_DIR $ENV{HOME}/src/darknet)
  endif()

  #message( STATUS "DEBUG: PATH_YOLO_DIR: " ${PATH_YOLO_DIR} )

  set(CUDNN_LIBS
    -L${CUDA_LIBRARY_DIR} -lcuda -lcudart -lcublas -lcurand)

  if(WITH_YOLOV4)
    set(YOLO_LIBRARIES
      ${YOLO_LIBRARIES}
      -L${PATH_YOLO_DIR} -ldarknet
      ${CUDA_CUDA_LIBRARY}
      ${CUDA_CUDART_LIBRARY}
      ${CUDNN_LIBS}
      )
  else()
    set(YOLO_LIBRARIES
      ${YOLO_LIBRARIES}
      ${PATH_YOLO_DIR}/libdarknet.a
      ${CUDA_CUDA_LIBRARY}
      ${CUDA_CUDART_LIBRARY}
      ${CUDNN_LIBS}
      )
  endif()

  include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${PATH_YOLO_DIR}/include
    ${PATH_YOLO_DIR}/src
    ${PATH_YOLO_DIR}/3rdparty/include
    )

endif()

add_service_files(
  FILES 
  SetMask.srv
  )

#add_action_files(DIRECTORY action FILES Sample.action)
add_message_files(
  FILES
  #Point2D32.msg
  ImageCoord32.msg
  #ICor32.msg
  #ClassifiedObstacle.msg
  #ClassifiedObstacleArray.msg
  Obstacle.msg
  ExpSetup.msg
  )

#generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES eSRS
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  message_runtime
  #  DEPENDS system_lib
)


add_subdirectory(src)
# subdirs(src/libsrss)

# add_executable(obstacle_classifier
#  src/YoloMapMask.cpp
#   src/obstacle_classifier.cpp
#   src/obstacle_detector.cpp)
# link_directories(${CUDA_LIBRARY_DIR})

# add_dependencies(obstacle_classifier
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS})

# target_link_libraries(obstacle_classifier
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${esrs_LIBRARIES}
# )


#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#  )

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#  )

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
