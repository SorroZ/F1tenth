cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation cv_bridge image_transport sensor_msgs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ncurses++ REQUIRED ncurses++)

## Declare ROS messages and services
add_message_files(FILES Num.msg driveCmd.msg driveMessage.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs beginner_tutorials)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Build all the packages
include_directories(include ${catkin_INCLUDE_DIRS} ${ncurses++_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#target_link_libraries ( vant ${ncurses++_LIBRARIES})
#target_link_libraries(${ncurses++_LIBRARIES})

# Specify the source files that are to be built, linking them with any needed libraries
add_executable(talkerim src/talkerim.cpp)
target_link_libraries(talkerim ${catkin_LIBRARIES})
add_dependencies(talkerim beginner_tutorials_generate_messages_cpp)

add_executable(teleop_traxxas src/teleop_traxxas.cpp)
target_link_libraries(teleop_traxxas ${catkin_LIBRARIES})
add_dependencies(teleop_traxxas beginner_tutorials_generate_messages_cpp)
