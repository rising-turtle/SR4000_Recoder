#
# CMake Toolchain file for crosscompiling on ARM.
#
# This can be used when running cmake in the following way:
#  cd build/
#  cmake .. -DCMAKE_TOOLCHAIN_FILE=../uClinux-toolchain.cmake

set(CROSS_PATH /home/david/work/EmbMess/opt/uClinux/bfin-uclinux)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER "${CROSS_PATH}/bin/bfin-uclinux-gcc")
set(CMAKE_CXX_COMPILER "${CROSS_PATH}/bin/bfin-uclinux-g++")

# message("STATUS: CMAKE_CXX_COMPILER : ${CMAKE_CXX_COMPILER}")

# these variables are set for the target system, when the target file runs, it knows where to find the dynamic libraries 

set(CMAKE_SYSROOT "${CROSS_PATH}")

# set(CMAKE_STAGING_PREFIX /home/devel/stage)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
