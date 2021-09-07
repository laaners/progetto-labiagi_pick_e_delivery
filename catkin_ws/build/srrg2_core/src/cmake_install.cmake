# Install script for directory: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/install")
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
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_system_utils/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_geometry/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_boss/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_property/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_config/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_image/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_pcl/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_matchable/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_data_structures/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_messages/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/srrg_viewer/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples/cmake_install.cmake")
  include("/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests/cmake_install.cmake")

endif()

