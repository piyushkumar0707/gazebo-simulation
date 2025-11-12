cmake_minimum_required(VERSION 3.0.2)
project(simple_3dof_arm)

find_package(catkin REQUIRED)

catkin_package()

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install worlds
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

# Install models
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

# Install scripts
install(PROGRAMS scripts/control_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
