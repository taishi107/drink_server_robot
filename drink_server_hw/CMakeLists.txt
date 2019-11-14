cmake_minimum_required(VERSION 2.8.3)
project(drink_server_hw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
	hardware_interface
	controller_manager
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES drink_server_hw
	CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_manager
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(diff_drive src/diff_drive.cpp src/dynamixel_hw.cpp )
add_executable(test_without_dynamixel src/diff_drive.cpp src/hw_test_without_dynamixel.cpp )

target_link_libraries(diff_drive
   ${catkin_LIBRARIES}
 )
target_link_libraries(test_without_dynamixel
   ${catkin_LIBRARIES}
 )

target_link_libraries(diff_drive
	${PROJECT_SOURCE_DIR}/include/drink_server_hw/dxlib.o
 )

