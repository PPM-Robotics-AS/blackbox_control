cmake_minimum_required(VERSION 2.8.3)
project(blackbox_control)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)

 #add_message_files(
 #  FILES
 #)

add_service_files(
   FILES
   GetAvailableRange.srv
   Playback.srv
   Export.srv
   Import.srv
)

generate_messages(
  DEPENDENCIES
   std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}-lib src/black_box_node.cpp)
target_link_libraries(${PROJECT_NAME}-lib ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gmock(utest test/black_box_node_test.cpp)
  target_link_libraries(utest ${catkin_LIBRARIES} ${PROJECT_NAME}-lib)
endif()

add_executable(${PROJECT_NAME} src/black_box_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
)

add_dependencies(
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(
  ${PROJECT_NAME}-lib
  ${PROJECT_NAME}_generate_messages_cpp
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
