# Install script for directory: /home/robond/ROS_Udacity/src/rtabmap/tools

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robond/ROS_Udacity/devel_isolated/rtabmap")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/ConsoleApp/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/ImagesJoiner/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/ExtractObject/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Camera/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/StereoEval/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/KittiDataset/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/RgbdDataset/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/EurocDataset/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Recovery/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Reprocess/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/DetectMoreLoopClosures/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Export/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Report/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Info/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/CameraRGBD/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/DatabaseViewer/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/EpipolarGeometry/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/OdometryViewer/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/DataRecorder/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Calibration/cmake_install.cmake")
  include("/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/tools/Matcher/cmake_install.cmake")

endif()

