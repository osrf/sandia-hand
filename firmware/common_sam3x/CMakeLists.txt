include_directories(../asf-3.5.1_partial/sam/utils/cmsis/sam3x/include)
include_directories(../asf-3.5.1_partial/thirdparty/CMSIS/Include)
add_definitions(-DF_CPU=64000000)
add_library(common_sam3x STATIC clock.c bootloader.c bootloader_startup.S)
