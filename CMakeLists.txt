cmake_minimum_required(VERSION 2.8.3)
project(wb_state_publisher)

find_package(catkin REQUIRED
  dwl
  dwl_msgs
  roscpp
  hardware_interface
  controller_interface
  control_toolbox
  realtime_tools
  pluginlib
  urdf
  dynamic_reconfigure
  tf)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp hardware_interface controller_interface pluginlib urdf tf
    dynamic_reconfigure dwl)


# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUIL_RELEASE Release)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${dwl_INCLUDE_DIRS})


## Declare a cpp library
add_library(${PROJECT_NAME}  src/WholeBodyStatePublisherController.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}
                                       ${dwl_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${dwl_msgs_EXPORTED_TARGETS})

install(DIRECTORY ${CMAKE_SOURCE_DIR}/config/
            DESTINATION DESTINATION share/${PROJECT_NAME}/config
            FILES_MATCHING PATTERN "*.yaml*")
install(DIRECTORY ${CMAKE_SOURCE_DIR}/launch/
            DESTINATION DESTINATION share/${PROJECT_NAME}/launch
            FILES_MATCHING PATTERN "*.*")
install(FILES ${CMAKE_SOURCE_DIR}/wb_state_publisher.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
            DESTINATION DESTINATION include
            FILES_MATCHING PATTERN "*.h*")
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
