cmake_minimum_required(VERSION 2.8.3)
project(catbot_description)

find_package(catkin REQUIRED roslaunch)
catkin_package()



install(DIRECTORY config images launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
