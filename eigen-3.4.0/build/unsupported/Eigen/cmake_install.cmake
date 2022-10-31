# Install script for directory: /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/AdolcForward"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/AlignedVector3"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/ArpackSupport"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/AutoDiff"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/BVH"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/EulerAngles"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/FFT"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/IterativeSolvers"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/KroneckerProduct"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/LevenbergMarquardt"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/MatrixFunctions"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/MoreVectorization"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/MPRealSupport"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/NonLinearOptimization"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/NumericalDiff"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/OpenGLSupport"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/Polynomials"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/Skyline"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/SparseExtra"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/SpecialFunctions"
    "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

