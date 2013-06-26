# ***************************
# CMake for graspWidget
# ***************************

# Create pure-ROS node for command joints
add_library( graspCmd GraspCmd.cpp )
target_link_libraries( graspCmd ${catkin_LIBRARIES} )

# Qt widgets
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions( ${QT_DEFINITIONS} )


# Set default path for executables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

set( qt_forms ui/graspInterface.ui )
set( qt_moc GraspWidget.h )

qt4_wrap_cpp( qt_moc_hpp ${qt_moc} )
qt4_wrap_ui( qt_forms_hpp ${qt_forms} )

set( qt_sources main.cpp GraspWidget.cpp ${qt_moc_hpp} ${qt_forms_hpp} )

# Set default path for executables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Set link directory
link_directories( ${CMAKE_SOURCE_DIR}/lib )

add_executable( graspWidget ${qt_sources} )
target_link_libraries( graspWidget ${QT_LIBRARIES} ${catkin_LIBRARIES} graspCmd )
