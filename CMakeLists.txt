cmake_minimum_required(VERSION 2.8.3)
project(rospilot_deps)
find_package(catkin 
	REQUIRED COMPONENTS 
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs 
        camera_info_manager)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib cv_bridge image_transport)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
pkg_check_modules(libavcodec REQUIRED libavcodec)
pkg_check_modules(libswscale REQUIRED libswscale)

set(ROS_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs
        camera_info_manager
)

catkin_python_setup()

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${libavcodec_INCLUDE_DIRS}
                    ${libswscale_INCLUDE_DIRS}
)
add_library(rospilot_deps
	src/usb_cam.cpp
)
target_link_libraries(rospilot_deps ${libavcodec_LIBRARIES} ${libswscale_LIBRARIES} ${catkin_LIBRARIES})

add_executable(raw_usb_cam_node nodes/raw_usb_cam_node.cpp)
target_link_libraries(raw_usb_cam_node rospilot_deps ${libavcodec_LIBRARIES} ${libswscale_LIBRARIES} ${catkin_LIBRARIES})
add_executable(raw_mjpeg_server src/raw_mjpeg_server.cpp)
target_link_libraries(raw_mjpeg_server
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS raw_usb_cam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS raw_mjpeg_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS rospilot_deps 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
