cmake_minimum_required(VERSION 2.8.3)

project(emulated_srs)

option(WITH_YOLO "With Yolo v3" ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -std=c++11")
   
set(CMAKE_VERBOSE_MAKEFILE 1)

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
set(esrs_INCDIRS)
set(esrs_LIBRARIES eSRS)

#find_package(OpenCV 2 REQUIRED)
#find_package(OpenCV 3 REQUIRED)
#if(NOT OpenCV_FOUND)
#  message(FATAL_ERROR "\n\n OpenCV 2 is missing\n\n")
#endif()
set(CMAKE_REQUIRED_INCLUDES
  ${CMAKE_REQUIRED_INCLUDES} ${OpenCV_INCLUDE_DIRS})
#set(esrs_LIBRARIES ${esrs_LIBRARIES} ${OpenCV_LIBS})
message( STATUS "DEBUG: OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS} )

if(WITH_YOLO)
  find_package(CUDA 10 REQUIRED)
  if(NOT CUDA_FOUND)
    message(FATAL_ERROR "\n\n CUDA 10 is missing\n\n")
  endif()
  set(CUDA_LIBRARY_DIR ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

  if(DEFINED $ENV{PATH_DARKNET})
    set(PATH_YOLO_DIR $ENV{PATH_DARKNET})
  else()
    set(PATH_YOLO_DIR $ENV{HOME}/src/darknet)
  endif()
  set(YOLO_LIBS ${PATH_YOLO_DIR}/libdarknet.a -fopenmp)
  set(CUDNN_LIBS -L${CUDA_LIBRARY_DIR} -lcuda -lcudart -lcublas -lcurand -lcudnn)
  set(YOLO_INCDIR ${PATH_YOLO_DIR}/include)

  set(esrs_INCDIRS ${esrs_INCDIRS} ${CUDA_INCLUDE_DIRS} ${YOLO_INCDIR})
  #set(esrs_LIBDIRS ${esrs_LIBDIRS} ${CUDA_LIBRARY_DIR})
  set(esrs_LIBRARIES ${esrs_LIBRARIES} ${YOLO_LIBS}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY}  ${CUDNN_LIBS})

  message(STATUS "DEBUG: esrs incdir: ${esrs_INCDIRS}")
  #message(STATUS "DEBUG: esrs libdir: ${esrs_LIBDIRS}")
  message(STATUS "DEBUG: esrs libs: ${esrs_LIBRARIES}")
endif()

#add_service_files(FILES WordCount.srv)

#add_action_files(DIRECTORY action FILES Sample.action)
add_message_files(
  FILES
  ClassifiedObstacle.msg
  ClassifiedObstacleArray.msg
  )

#generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES emulated_srs
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${esrs_INCDIRS}
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

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#  )

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
