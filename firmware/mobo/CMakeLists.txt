set(OPENOCD_INIT ${OPENOCD} -f ${CMAKE_CURRENT_SOURCE_DIR}/jtagkey.cfg -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd.cfg -c init -c halt)

add_subdirectory(mcu)
add_subdirectory(fpga)


