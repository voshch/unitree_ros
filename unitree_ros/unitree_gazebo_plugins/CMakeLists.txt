cmake_minimum_required(VERSION 3.0.2)
project(unitree_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS gazebo_ros gazebo_msgs )
find_package(gazebo REQUIRED )
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(unitreeFootContactPlugin SHARED plugins/foot_contact_plugin.cc)
target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugins/draw_force_plugin.cc)
target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})