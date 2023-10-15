cmake_minimum_required(VERSION 2.8.3)
project(vineyard_color_filter)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
  	image_transport
	roscpp
  	sensor_msgs
	tf
	dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
	config/ColorFilterParameters.cfg
)

find_package(OpenCV REQUIRED)

add_compile_options(-std=c++11)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ransac_pointcloud2
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories( include 
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS} )

add_executable(vineyard_color_filter
  src/main.cpp
  src/VineyardColorFilter.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(
	vineyard_color_filter
  	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)

