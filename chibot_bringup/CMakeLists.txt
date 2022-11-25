cmake_minimum_required(VERSION 3.0.2)
project(chibot_bringup)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
)


catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_odom src/chibot_odom.cpp)
add_executable(${PROJECT_NAME}_conn src/chibot_connect.cpp)

target_link_libraries(${PROJECT_NAME}_odom
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_conn
  ${catkin_LIBRARIES}
)
