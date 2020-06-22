cmake_minimum_required(VERSION 2.8.3)
project(armcontrolmoveit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  interactive_markers
  geometric_shapes
  tf2_ros
  message_generation
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  PoseArrayStamped.msg
)

add_service_files(
  FILES
  ### ArmControl
  ChangeTarget.srv
  ExecuteTarget.srv
  DemoPrecision.srv
  HomeService.srv
  PlanTrajectory.srv
  ### Door
  GenerarTrayectoriaPuerta.srv
  GirarSistemaPuerta.srv
  PosicionApoyo.srv
  DrawDoorRViz.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
catkin_package(
  INCLUDE_DIRS include ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES armcontrolmoveit
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib EIGEN3
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

#############################################################################
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

######################## Declaracion de nodos ##########################

## Nodo para inicializacion de servicios del brazo
add_executable(
  ### Nombre del nodo
  arm_services

  ### Ficheros que usa el nodo
  src/ArmServices.cpp 
  src/ArmControl.cpp 
  src/VisualTools.cpp 
  src/Door.cpp 
  src/UsefulFunctions.cpp
)
target_link_libraries(arm_services ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS arm_services DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Nodo para inicializacion de servicios de la puerta
add_executable(
  ### Nombre del nodo
  door_services 

  ### Ficheros que usa el nodo
  src/DoorServices.cpp 
  src/VisualTools.cpp 
  src/Door.cpp 
  src/ArmControl.cpp 
  src/UsefulFunctions.cpp
)
target_link_libraries(door_services ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS door_services DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Nodo de pruebas
add_executable(
  ### Nombre del nodo
  pruebas_trayectorias

  ### Ficheros que usa el nodo
  src/pruebas_trayectorias.cpp
  src/VisualTools.cpp
  src/Door.cpp
  src/ArmControl.cpp
  src/UsefulFunctions.cpp
)
target_link_libraries(pruebas_trayectorias ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pruebas_trayectorias DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
