cmake_minimum_required(VERSION 3.0.2)
project(rosbag_panel)
find_package(catkin REQUIRED COMPONENTS rviz rosbag cv_bridge)
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS rviz rosbag)
include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_autogen/include)
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS resource)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
    src/rosbag_panel.cpp
    src/rosbag_tool.cpp
    src/treemodel.cpp
    src/range_slider.cpp
    src/image_dialog.cpp
    src/filtered_bag_generation.cpp
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rosbag_panel", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``.
add_library(${PROJECT_NAME} ${SRC_FILES})

add_dependencies(${PROJECT_NAME} rosbag_generate_messages_cpp)


## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
##
## Although this puts "rosbag_panel" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})