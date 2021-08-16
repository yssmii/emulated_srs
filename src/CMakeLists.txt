cmake_minimum_required(VERSION 2.8)

add_definitions(-DHAVE_CONFIG_H)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

#message(STATUS "DEBUG: SRSS INCDIRS: ${esrs_INCDIRS}")

include_directories(
  lib/include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(obstacle_detector
  intensity_map.cpp
  obstacle_detector.cpp
  obstacle_detector_main.cpp)

add_executable(obstacle_classifier
  yolo_map.cpp
  intensity_map.cpp
  obstacle_detector.cpp
  obstacle_classifier.cpp)

add_executable(obstacle_measurer
  intensity_map.cpp
  obstacle_detector.cpp
  obstacle_measurer.cpp)

add_executable(region_maker
  intensity_map.cpp
  obstacle_detector.cpp
  region_maker.cpp)

link_directories(${CUDA_LIBRARY_DIR})

add_dependencies(obstacle_detector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(obstacle_classifier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(obstacle_measurer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(region_maker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

#message(STATUS "DEBUG: OpenCV LIBRARIES: ${OpenCV_LIBRARIES}")
#message(STATUS "DEBUG: esrs LIBRARIES: ${esrs_LIBRARIES}")

target_link_libraries(obstacle_detector
  eSRS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YOLO_LIBRARIES}
  )

target_link_libraries(obstacle_classifier
  eSRS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YOLO_LIBRARIES}
  )

target_link_libraries(obstacle_measurer
  eSRS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YOLO_LIBRARIES}
  )

target_link_libraries(region_maker
  eSRS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YOLO_LIBRARIES}
  )

add_custom_target(maskfile)
add_custom_command(TARGET obstacle_detector POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/share/MASK.png $<TARGET_FILE_DIR:obstacle_detector>
  )

add_custom_command(TARGET obstacle_classifier POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/share/MASK.png $<TARGET_FILE_DIR:obstacle_classifier>
  )

add_custom_command(TARGET obstacle_measurer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/share/MASK.png $<TARGET_FILE_DIR:obstacle_measurer>
  )

add_custom_command(TARGET region_maker POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/share/MASK.png $<TARGET_FILE_DIR:region_maker>
  )

if(WITH_YOLO)
  add_custom_command(TARGET obstacle_classifier POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:obstacle_classifier>/data
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/share/coco.names $<TARGET_FILE_DIR:obstacle_classifier>/data
    )
endif(WITH_YOLO)
  
add_subdirectory(lib)
add_subdirectory(util)
#add_subdirectory(share)

