#
# CMake Toolchain file for crosscompiling on ARM.
#
# This can be used when running cmake in the following way:
#  cd build/
#  cmake .. -DCMAKE_TOOLCHAIN_FILE=../cross-arm-linux-gnueabihf.cmake

set(CROSS_PATH /opt/gcc-linaro-arm-linux-gnueabihf-4.7-2013.02-01-20130221_linux)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER "${CROSS_PATH}/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${CROSS_PATH}/bin/arm-linux-gnueabihf-g++")

# message("STATUS: CMAKE_CXX_COMPILER : ${CMAKE_CXX_COMPILER}")

# these variables are set for the target system, when the target file runs, it knows where to find the dynamic libraries 

set(CMAKE_SYSROOT "${CROSS_PATH}")

# set(CMAKE_STAGING_PREFIX /home/devel/stage)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
