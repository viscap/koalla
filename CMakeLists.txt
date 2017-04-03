cmake_minimum_required(VERSION 2.8.3)
project(koalla)

#set(CMAKE_PREFIX_PATH "/usr/local/lib")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  mavros_msgs
  sensor_msgs
  dynamic_reconfigure
 )

find_package(OpenCV REQUIRED)


####### dynamic reconfigure #######
###################################
generate_dynamic_reconfigure_options(
  cfg/Koalla.cfg
)


#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
    mavros_msgs
    sensor_msgs
    dynamic_reconfigure
)


include_directories(
	include ${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS} 
	/usr/local/include/
)


###########
## Build ##
###########

add_executable(${PROJECT_NAME} src/image_subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} 	${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})


add_executable(koalla_old src/image_subscriber_old.cpp)
target_link_libraries(koalla_old ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(koalla_old ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(camera_driver src/camera_driver.cpp)
target_link_libraries(camera_driver ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(camera_driver ${PROJECT_NAME})


add_executable(koalla_ruan src/image_subscriber_ruan.cpp)
add_dependencies(koalla_ruan ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(koalla_ruan ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
