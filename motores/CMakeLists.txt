cmake_minimum_required(VERSION 2.8.3)
project(motores)

find_package(catkin REQUIRED COMPONENTS dynamixel_sdk roscpp std_msgs)

catkin_package(CATKIN_DEPENDS dynamixel_sdk roscpp)

include_directories( ${dynamixel_sdk_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS}       )

add_executable(motores src/motores.cpp)
target_link_libraries(motores ${catkin_LIBRARIES}
                           ${dynamixel_sdk_LIBRARIES})

add_executable(std_pose src/std_pose.cpp)
target_link_libraries(std_pose ${catkin_LIBRARIES}
                           ${dynamixel_sdk_LIBRARIES})

add_executable(pingMotores src/pingMotores.cpp)
target_link_libraries(pingMotores ${catkin_LIBRARIES}
                           ${dynamixel_sdk_LIBRARIES})
