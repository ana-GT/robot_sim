# ***************************
# CMake for graspWidget
# ***************************

# Create pure-ROS node for command joints
add_library( gainsTuningCmd GainsTuningCmd.cpp )
target_link_libraries( gainsTuningCmd ${catkin_LIBRARIES} )

# Qt widgets
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions( ${QT_DEFINITIONS} )


# Set default path for executables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

set( qt_forms ui/gainsTuningInterface.ui )
set( qt_moc GainsTuningWidget.h )

qt4_wrap_cpp( qt_moc_hpp ${qt_moc} )
qt4_wrap_ui( qt_forms_hpp ${qt_forms} )

set( qt_sources main.cpp GainsTuningWidget.cpp ${qt_moc_hpp} ${qt_forms_hpp} ) 

# Set default path for executables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Set link directory
link_directories( ${CMAKE_SOURCE_DIR}/lib )

add_executable( gainsTuning ${qt_sources} )
target_link_libraries( gainsTuning ${QT_LIBRARIES} ${catkin_LIBRARIES} gainsTuningCmd ) 
