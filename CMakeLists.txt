cmake_minimum_required(VERSION 2.8.3)
project(bb_state)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg tf)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


add_message_files(DIRECTORY msg FILES TwistWithID.msg State.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Generate messages in the 'msg' folder

catkin_package(
	DEPENDS roscpp 
	INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} ../EposManager/)

add_executable(state_machine src/StateMachine.cpp src/IONode.cpp)
target_link_libraries(state_machine ${catkin_LIBRARIES})
add_dependencies(state_machine bb_state_gencpp )

# add_executable(io_node src/IONode.cpp)
# target_link_libraries(io_node ${catkin_LIBRARIES})
# add_dependencies(io_node bb_state_gencpp)

