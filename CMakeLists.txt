cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  tf2
  tf2_ros
  rospy
  dynamic_reconfigure
)

find_package(orocos_kdl REQUIRED)

############################
## dynamic reconfiure api ##
############################
generate_dynamic_reconfigure_options(
  cfg/PurePursuit.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs ackermann_msgs tf2 tf2_ros rospy dynamic_reconfigure
  )


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pure_pursuit src/pure_pursuit.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Make sure configure headers are built before any node using them
add_dependencies(pure_pursuit ${PROJECT_NAME}_gencfg)
  

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/figure_eight.py scripts/simple_loop.py 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(TARGETS pure_pursuit
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

