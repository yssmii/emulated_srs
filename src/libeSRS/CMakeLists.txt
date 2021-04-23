cmake_minimum_required(VERSION 2.8)

#PROJECT(libvbpd CXX)

# config.h
add_definitions(-DHAVE_CONFIG_H)
include(CheckIncludeFileCXX)

message(STATUS "CMAKE_REQUIRED_INCLUDES: ${CMAKE_REQUIRED_INCLUDES}")
message(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

check_include_file_cxx(unistd.h HAVE_UNISTD_H)
check_include_file_cxx(time.h HAVE_TIME_H)
check_include_file_cxx("sys/time.h" HAVE_SYS_TIME_H)
check_include_file_cxx(termios.h HAVE_TERMIOS_H)
check_include_file_cxx(fcntl.h HAVE_FCNTL_H)
check_include_file_cxx(conio.h HAVE_CONIO_H)
check_include_file_cxx(winsock2.h HAVE_WINSOCK2_H)
check_include_file_cxx(sys/socket.h HAVE_SYS_SOCKET_H)
check_include_file_cxx("X11/keysym.h" HAVE_X11_KEYSYM_H)
check_include_file_cxx(pthread.h HAVE_PTHREAD_H)
#check_include_file_cxx(cv.h HAVE_CV_H)
#check_include_file_cxx("opencv/cv.h" HAVE_OPENCV_CV_H)
#check_include_file_cxx(highgui.h HAVE_HIGHGUI_H)
#check_include_file_cxx("opencv/highgui.h" HAVE_OPENCV_HIGHGUI_H)
check_include_file_cxx(OpenNI.h HAVE_OPENNI_H)
check_include_file_cxx("openni2/OpenNI.h" HAVE_OPENNI2_OPENNI_H)

if(HAVE_UNISTD_H)
  message(STATUS "FOUND: unistd.h") # test
endif(HAVE_UNISTD_H)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

# include dirs
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# library
set(SERIAL "2.0.0")
set(SOSERIAL "2")

#file(GLOB ESRS_SRCS ${CMAKE_CURRENT_SOURCE_DIR} eSRS/*.cpp UFV/*.cpp)
#FILE(GLOB ESRS_SRCS *.cpp UFV/*.cpp)

set(ESRS_SRCS
  eSRS/DepthMap.cpp
  eSRS/DetectionMap.cpp
  eSRS/RangeData.cpp
  eSRS/ClassificationMap.cpp
  UFV/utils.cpp
  UFV/CameraParam.cpp
  UFV/ImageData.cpp
  UFV/IntensityMap.cpp
  UFV/RangeData.cpp)
if(WITH_YOLO)
  include_directories(${PATH_YOLO_DIR}/include ${PATH_YOLO_DIR}/src
    ${PATH_YOLO_DIR}/3rdparty/include)
  set(ESRS_SRCS ${ESRS_SRCS} eSRS/YoloMap.cpp)
endif(WITH_YOLO)

add_library(eSRS ${LIB_TYPE} ${ESRS_SRCS})

target_link_libraries(eSRS ${OpenCV_LIBS})

set_target_properties(eSRS PROPERTIES VERSION ${SERIAL} SOVERSION ${SOSERIAL})

install(TARGETS eSRS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
