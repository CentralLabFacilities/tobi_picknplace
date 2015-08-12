cmake_minimum_required(VERSION 2.8.3)
project(tobi_picknplace)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    moveit_ros_planning_interface
    actionlib
    shape_tools
    shape_msgs
    moveit_msgs
    visualization_msgs
    cmake_modules
    katana_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
    message_runtime
    roscpp
    tf2
    moveit_ros_planning_interface
    actionlib
    shape_tools
    shape_msgs
    moveit_msgs
    visualization_msgs
    katana_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})

## find Eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## find RSB
find_package(RST 0.11 REQUIRED)
find_package(RSB 0.11 REQUIRED)
find_package(RSC 0.11 REQUIRED)
INCLUDE_DIRECTORIES(${RSB_INCLUDE_DIRS} ${RSC_INCLUDE_DIRS} ${RST_INCLUDE_DIRS})
LINK_DIRECTORIES(${RSB_LIBRARY_DIRS} ${RSC_LIBRARY_DIRS} ${RST_LIBRARY_DIRS})
SET(ALL_RSB_LIBRARIES ${RSB_LIBRARIES} ${RSC_LIBRARIES} ${RST_LIBRARIES})
ADD_DEFINITIONS(${RST_CFLAGS})

###########
## Build ##
###########

## Declare a cpp executable
add_executable(tobi_picknplace_server_node 
  src/picknplace_server.cpp
  src/control/Controller.cpp
  src/control/ViaPoseStrategy.cpp
  src/interface/RsbInterface.cpp
  src/interface/ViewInterface.cpp
  src/model/KatanaModel.cpp
  src/model/KatanaSimModel.cpp
  src/model/GraspGenerator.cpp
  src/util/TransitionReader.cpp
  src/util/TransformerTF.cpp
  src/util/Dijkstra.cpp
  src/util/RosTools.cpp
  src/util/ParamReader.cpp
)
add_executable(tobi_picknplace_client
  src/picknplace_client.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(tobi_picknplace_server_node "")

## Specify libraries to link a library or executable target against
target_link_libraries(tobi_picknplace_server_node
  ${catkin_LIBRARIES}
  ${ALL_RSB_LIBRARIES}
)
target_link_libraries(tobi_picknplace_client
  ${catkin_LIBRARIES}
  ${ALL_RSB_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS tobi_picknplace_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS tobi_picknplace_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
file(GLOB_RECURSE KATANA_PROGRAMS scripts/katana-*)
install(PROGRAMS ${KATANA_PROGRAMS}
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
