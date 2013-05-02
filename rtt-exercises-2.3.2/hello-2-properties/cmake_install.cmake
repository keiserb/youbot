# Install script for directory: /home/keiserb/myproject/rtt-exercises-2.3.2/hello-2-properties

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux"
         RPATH "/usr/local/bin:/usr/local/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/rtt/install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/rtt/install/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/keiserb/myproject/rtt-exercises-2.3.2/hello-2-properties/HelloWorld-gnulinux")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux"
         OLD_RPATH "/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/rtt/install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/rtt/install/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/lib::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/bin:/usr/local/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/rtt/install/lib:/opt/ros/fuerte/stacks/orocos_toolchain/rtt/install/lib/orocos/gnulinux/plugins:/opt/ros/fuerte/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/HelloWorld-gnulinux")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/keiserb/myproject/rtt-exercises-2.3.2/hello-2-properties/hello-2-properties-gnulinux.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/keiserb/myproject/rtt-exercises-2.3.2/hello-2-properties/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/keiserb/myproject/rtt-exercises-2.3.2/hello-2-properties/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
