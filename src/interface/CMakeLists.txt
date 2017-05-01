##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(interface)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

##############################################################################
# Catkin
##############################################################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/cmake/options.cmake)
include_directories(
        ${OPLK_BASE_DIR}/stack/proj/linux/liboplkmndrv-pcap
        ${OPLK_BASE_DIR}/apps/common/objdicts/CiA302-4_MN
)

# protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
message(STATUS ${PROTOBUF_INCLUDE_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/protobuf/myoFPGA.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp communication roboy_managing_node)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/powerlink/output)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(CATKIN_DEPENDS roboy_managing_node communication )
include_directories(${catkin_INCLUDE_DIRS})

include_directories(qcustomplot)

find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml REQUIRED tinyxml)
include_directories(${tinyxml_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/interface/*.hpp qcustomplot/qcustomplot.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp qcustomplot/qcustomplot.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(interface ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} )
target_link_libraries(interface ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} roboy_managing_node ${tinyxml_LIBRARIES} )
##############################################################################
# Installation
##############################################################################
install(TARGETS interface RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

