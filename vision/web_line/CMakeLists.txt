find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(white_and_black whiteLine.cpp white_and_black.cpp)
target_link_libraries(white_and_black ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(white_and_black ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
