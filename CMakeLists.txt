cmake_minimum_required(VERSION 2.8.3)
project(robot_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED
	COMPONENTS message_generation std_msgs roscpp )


# Find gazebo
include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
else()
  message( FATAL_ERROR "pkg-config is required, please install it" )
endif()


# Declare the message files to be build
add_message_files( FILES robotState.msg JointCommands.msg )
# Generate the language-specific message (and service files, if any)
generate_messages( DEPENDENCIES std_msgs )


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy
    DEPENDS gazebo )

include_directories( include )

## Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate the plugins
add_subdirectory( plugins )

# Generate nodes
add_subdirectory( src )
