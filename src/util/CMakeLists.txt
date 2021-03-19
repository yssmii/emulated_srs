add_definitions(-DHAVE_CONFIG_H)
include_directories(BEFORE ../libeSRS)

# makeMask
#ADD_EXECUTABLE(makeMask makeMask.cpp opt/getopt.c)
add_executable(makeMask makeMask.cpp)
target_link_libraries(makeMask ${OpenCV_LIBRARIES})
