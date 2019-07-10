cmake_minimum_required(VERSION 2.8.3)
project(cob_hand_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  control_msgs
  diagnostic_updater
  message_generation
  sensor_msgs
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS chrono system thread)

add_message_files(
  FILES
  JointValues.msg
  Status.msg
)

add_service_files(
  FILES
  InitFinger.srv
  InitPins.srv
  SetPWM.srv
  UpdatePins.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cob_hand_bridge
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  client
  client/src
  client/pigpio
)

add_executable(${PROJECT_NAME}_node src/cob_hand_bridge_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(cob_hand_pigpio STATIC client/pigpio/pigpio.c client/pigpio/command.c)
target_compile_definitions(cob_hand_pigpio PRIVATE DISABLE_SER_CHECK_INITED)

add_executable(cob_hand_node src/cob_hand_node.cpp)
target_link_libraries(cob_hand_node cob_hand_pigpio ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(cob_hand_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_hand test/test_hand.cpp)
target_link_libraries(test_hand cob_hand_pigpio ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(test_hand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


### INSTALL ###
install(TARGETS ${PROJECT_NAME}_node cob_hand_node test_hand
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
