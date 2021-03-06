cmake_minimum_required(VERSION 2.8.3)
project(hhi_autoexcavator)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  dynamic_reconfigure
  
  message_generation
  
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)

generate_dynamic_reconfigure_options(
  cfg/param_reconfigure.cfg
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  hhi_command.msg
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS message_runtime roscpp dynamic_reconfigure
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  )

set(SRC ${SRC} src/main.cpp)

set(DCMAKE_build_type release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")

add_executable (hhi_gcs ${SRC} )
target_link_libraries (hhi_gcs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_dependencies(hhi_gcs ${PROJECT_NAME}_gencfg)
