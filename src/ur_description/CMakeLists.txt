# you copy from /opt/ros/humble..., there's no cmakelists
# you need to create one on your own
# https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(ur_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

ament_package()
