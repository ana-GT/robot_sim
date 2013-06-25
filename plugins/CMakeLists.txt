# *****************************************************
# Add DRCPlugin library
# WorldPlugin similar to VRC Plugin, but for DRC-Hubo
# *****************************************************

include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_libraries( ${GAZEBO_LIBRARY_DIRS} )

# drchuboPlugin: Model Plugin
add_library(drchuboPlugin drchuboPlugin.cpp)
target_link_libraries(drchuboPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# drcsim_plugins : Snatched out of drcsim
add_library( gazebo_ros_api_plugin drcsim_plugins/gazebo_ros_api_plugin.cpp )
target_link_libraries( gazebo_ros_api_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


add_library(worldSimPlugin worldSimPlugin.cpp)
target_link_libraries(worldSimPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


