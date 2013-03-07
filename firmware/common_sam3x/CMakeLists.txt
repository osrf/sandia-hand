set(CMAKE_C_LINK_FLAGS ${SAM3X_APPLICATION_FLAGS})
add_library(common_sam3x STATIC console.c clock.c bootloader.c bootloader_startup.S)
