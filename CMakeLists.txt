cmake_minimum_required(VERSION 2.8.3)
project(find_color)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Ellipse.msg
  Ellipses.msg
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
		CATKIN_DEPENDS cv_bridg image_transport roscpp rospy sensor_msgs std_mags
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package( OpenCV REQUIRED )
add_executable(find_bgr src/find_bgr.cpp)
add_dependencies(find_bgr ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(find_bgr ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  )

add_executable(find_hsv src/find_hsv.cpp)
add_dependencies(find_hsv ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(find_hsv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  )



