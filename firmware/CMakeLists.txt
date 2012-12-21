cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sandia-hand-firmware)

set (ARM_GCC_ROOT ${CMAKE_CURRENT_BINARY_DIR}/arm-2011.09)

if (NOT EXISTS ${ARM_GCC_ROOT}.tar.bz2)
  message("Downloading ARM cross-compiler...")
  file(DOWNLOAD https://sourcery.mentor.com/sgpp/lite/arm/portal/package9740/public/arm-none-eabi/arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2 
       ${ARM_GCC_ROOT}.tar.bz2
       SHOW_PROGRESS)
endif()

add_custom_command(OUTPUT ${ARM_GCC_ROOT}
                   COMMAND tar xjf ${ARM_GCC_ROOT}.tar.bz2
                   DEPENDS ${ARM_GCC_ROOT}.tar.bz2)
add_custom_target(arm-gcc_extraction
                  ALL DEPENDS ${ARM_GCC_ROOT})

set(CMAKE_SYSTEM_NAME arm_baremetal)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_C_COMPILER ${ARM_GCC_ROOT}/bin/arm-none-eabi-gcc)
set(CMAKE_FIND_ROOT_PATH ${ARM_GCC_ROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) # search only host
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)  # search only target
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)  # search only target

# now clone openocd repo and get it up and running
set(OPENOCD_PATH ${CMAKE_CURRENT_BINARY_DIR}/openocd)
add_custom_command(OUTPUT ${OPENOCD_PATH}
                   COMMAND git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd ${OPENOCD_PATH})
add_custom_target(openocd_clone 
                  ALL DEPENDS ${OPENOCD_PATH})

#set(OPENOCD 
#if (NOT EXISTS ${
#add_custom_command(OUTPUT 



#add_subdirectory(common)
