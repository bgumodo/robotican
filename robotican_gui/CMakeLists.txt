##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robotican_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
include_directories(src)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################
# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/images/image_resource_file.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robotican_gui/*.hpp)



set(CMAKE_AUTORCC ON)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC} src/event_handler/EventSignal.h src/event_handler/EventSlot.h)
QT4_ADD_RESOURCES(RIC_GUI_RESOURCES_RCC resources/images/image_resource_file.qrc)
##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
set(PROJ_FILES
        src/main.cpp
        src/gui_manager/GUIManager.cpp
        src/gui_manager/GUIManager.h
        src/event_handler/EventSlot.cpp
        src/event_handler/EventSlot.h
        src/event_handler/EventSignal.cpp
        src/event_handler/EventSignal.h
        src/gui_components/Led.cpp
        src/gui_components/Led.h
        include/robotican_gui.h
        src/listeners/Listener.h




        #listeners
        src/listeners/Battery.cpp
        src/listeners/Battery.h
        src/listeners/Arm.cpp
        src/listeners/Arm.h
        src/listeners/PanTilt.cpp
        src/listeners/PanTilt.h
        src/listeners/Odom.cpp
        src/listeners/Odom.h
        src/listeners/Gripper.cpp
        src/listeners/Gripper.h
        src/listeners/Imu.cpp
        src/listeners/Imu.h
        src/listeners/Lidar.cpp
        src/listeners/Lidar.h
        src/listeners/Gps.cpp
        src/listeners/Gps.h
        src/listeners/FrontCam.cpp
        src/listeners/FrontCam.h
        src/listeners/RearCam.cpp
        src/listeners/RearCam.h
        src/listeners/UrfLeft.cpp
        src/listeners/UrfLeft.h
        src/listeners/UrfRear.cpp
        src/listeners/UrfRear.h
        src/listeners/UrfRight.cpp
        src/listeners/UrfRight.h
        src/listeners/Kinect2.cpp
        src/listeners/Kinect2.h
        src/listeners/F200.cpp
        src/listeners/F200.h
        )

add_executable(robotican_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${PROJ_FILES} ${RIC_GUI_RESOURCES_RCC})
target_link_libraries(robotican_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS robotican_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

