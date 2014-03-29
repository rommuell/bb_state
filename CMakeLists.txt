cmake_minimum_required(VERSION 2.8.3)
project(bbcontrol)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg tf)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES TwistWithID.msg Rake.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)



catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(nodeforcircle src/nodeforcircle.cpp)
target_link_libraries(nodeforcircle ${catkin_LIBRARIES})
add_dependencies(nodeforcircle nodeforcircle_gencpp)

add_executable(state_machine src/state_machine.cpp)
target_link_libraries(state_machine ${catkin_LIBRARIES})
add_dependencies(state_machine bbcontrol_gencpp)

add_executable(TestTwistpublisher src/TestTwistpublisher.cpp)
target_link_libraries(TestTwistpublisher ${catkin_LIBRARIES})
add_dependencies(TestTwistpublisher TestTwistpublisher_gencpp)


add_executable(test_cont  src/node_wolf.cpp)
target_link_libraries(test_cont ${catkin_LIBRARIES})
add_dependencies(test_cont test_cont_gencpp)

add_executable(node_wolf  src/node_wolf.cpp)
target_link_libraries(node_wolf ${catkin_LIBRARIES})
add_dependencies(node_wolf node_wolf_gencpp)

add_executable(JoystickoneStickversion  src/JoystickoneStickversion.cpp)
target_link_libraries(JoystickoneStickversion ${catkin_LIBRARIES})
add_dependencies(JoystickoneStickversion bbcontrol_gencpp)

add_executable(io_node  src/io_node.cpp)
target_link_libraries(io_node ${catkin_LIBRARIES})
add_dependencies(io_node io_node_gencpp)

catkin_package()


