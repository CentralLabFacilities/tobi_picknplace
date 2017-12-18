cmake_minimum_required(VERSION 2.8.3)
project(biron_posture_execution_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   BironPostureExecution.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
  DEPENDS
)
