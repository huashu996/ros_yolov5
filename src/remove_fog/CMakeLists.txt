cmake_minimum_required(VERSION 3.0.2)
project(remove_fog)


find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs roscpp rospy std_msgs)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rem_fog
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS scripts/remove_fog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
