# ***************************
# CMake for graspWidget
# ***************************

# Create pure-ROS node for command joints
add_executable( pubTest pubTest.cpp )
target_link_libraries( pubTest ${catkin_LIBRARIES} )


