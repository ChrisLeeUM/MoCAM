cmake_minimum_required(VERSION 2.8.3)
project(ai_control)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy)

  catkin_python_setup()

  catkin_package()

  include_directories(${catkin_INCLUDE_DIRS})

  catkin_install_python(PROGRAMS
            src/cmd_interface.py
            src/route_navi.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
