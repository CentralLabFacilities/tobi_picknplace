cmake_minimum_required(VERSION 2.8.3)
project(tobi_picknplace)

option(WITH_KATANA "Use the katana gripper" OFF)
option(USE_H2R5 "Use the h2r5 hand" ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CTKN_PKGS
    roscpp
    tf2
    moveit_ros_planning_interface
    actionlib
    geometric_shapes
    shape_msgs
    grasp_viewer
    moveit_msgs
    visualization_msgs
    grasping_msgs
)

set(CTKN_CMPNTS
    ${CTKN_PKGS}
    tf2_ros
    cmake_modules
)
if(WITH_KATANA)
  add_definitions(-DUSE_KATANA=ON)
endif(WITH_KATANA)

if(WITH_KATANA)
    find_package(catkin REQUIRED COMPONENTS
        ${CTKN_CMPNTS}
        katana_msgs
    )
else(WITH_KATANA)
    find_package(catkin REQUIRED COMPONENTS
        ${CTKN_CMPNTS}
    )
endif(WITH_KATANA)

include_directories(${catkin_INCLUDE_DIRS})

if(WITH_KATANA)
    catkin_package(CATKIN_DEPENDS
        ${CTKN_PKGS}
        message_runtime
        katana_msgs
    )
else(WITH_KATANA)
    catkin_package(CATKIN_DEPENDS 
        message_runtime
        ${CTKN_PKGS}
)
endif(WITH_KATANA)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})

## find Eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## find RSB
find_package(RST 0.13 REQUIRED)
find_package(RSB 0.13 REQUIRED)
find_package(RSC 0.13 REQUIRED)
INCLUDE_DIRECTORIES(${RSB_INCLUDE_DIRS} ${RSC_INCLUDE_DIRS} ${RST_INCLUDE_DIRS})
LINK_DIRECTORIES(${RSB_LIBRARY_DIRS} ${RSC_LIBRARY_DIRS} ${RST_LIBRARY_DIRS})
SET(ALL_RSB_LIBRARIES ${RSB_LIBRARIES} ${RSC_LIBRARIES} ${RST_LIBRARIES})
ADD_DEFINITIONS(${RST_CFLAGS})

###########
## Build ##
###########

set(DEFAULT_FILES 
  src/picknplace_server.cpp
  src/control/Controller.cpp
  src/control/ViaPoseStrategy.cpp
  src/grasping/GraspGenerator.h
  src/grasping/CentroidGrasping.cpp
  src/interface/AGNIInterface.cpp
  src/interface/RsbInterface.cpp
  src/interface/ViewInterface.cpp
  src/model/ModelFactory.cpp
  src/util/TransitionReader.cpp
  src/util/TransformerTF.cpp
  src/util/Dijkstra.cpp
  src/util/RosTools.cpp
  src/util/ParamReader.cpp
  src/model/Model.cpp
  src/model/types/H2R5.cpp
)

## Declare a cpp executable

if(WITH_KATANA)
    add_executable(tobi_picknplace_server_node
      ${DEFAULT_FILES}
      src/model/types/Katana.cpp
      src/model/types/KatanaSim.cpp
    )
else(WITH_KATANA)
    add_executable(tobi_picknplace_server_node
      ${DEFAULT_FILES}
    )
endif(WITH_KATANA)

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

#################
## style guide ##
#################
set(ENABLE_CPPLINT 1)
include(${CMAKE_CURRENT_SOURCE_DIR}/stylecheck/CpplintWrapper.cmake)
CPPLINT_RECURSIVE(cpplint_src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/
  ${CMAKE_CURRENT_BINARY_DIR}/src)
add_dependencies(tobi_picknplace_server_node cpplint_src)


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

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(WITH_KATANA)
    file(GLOB_RECURSE KATANA_PROGRAMS scripts/katana-*)
    install(PROGRAMS ${KATANA_PROGRAMS}
      DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endif(WITH_KATANA)
