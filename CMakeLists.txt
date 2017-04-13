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

# Versao mais atual do programa
add_executable(koalla src/koalla.cpp)
target_link_libraries(koalla ${catkin_LIBRARIES} 	${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(koalla ${PROJECT_NAME})

# Versao que foi enviada para o ICAR
add_executable(koalla_icar src/koalla_icar.cpp)
target_link_libraries(koalla_icar ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(koalla_icar ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# Versao de backup
add_executable(koalla_backup src/koalla_backup.cpp)
target_link_libraries(koalla_backup ${catkin_LIBRARIES} 	${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(koalla_backup ${PROJECT_NAME})

# Versao do Ruan com threshold adaptativo
add_executable(adaptive_threshold src/adaptive_threshold.cpp)
target_link_libraries(adaptive_threshold ${catkin_LIBRARIES} 	${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(adaptive_threshold ${PROJECT_NAME})

# Versao com reconfiguracao dinamica do valor do threshold
add_executable(threshold_with_reconfigure src/threshold_with_reconfigure.cpp)
target_link_libraries(threshold_with_reconfigure ${catkin_LIBRARIES} 	${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(threshold_with_reconfigure ${PROJECT_NAME})

# Driver para comunicar com a camera
add_executable(camera_driver src/camera_driver.cpp)
target_link_libraries(camera_driver ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} )
add_dependencies(camera_driver ${PROJECT_NAME})
