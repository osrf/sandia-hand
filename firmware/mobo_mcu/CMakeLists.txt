add_executable(mobo_mcu.elf mobo_mcu.c startup.c)
target_link_libraries(mobo_mcu.elf common_sam3x)
set_target_properties(mobo_mcu.elf PROPERTIES LINK_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/../common_sam3x/sam3x8e.ld")

set(OPENOCD_INIT ${OPENOCD} -f ${CMAKE_CURRENT_SOURCE_DIR}/jtagkey.cfg -f ${CMAKE_CURRENT_SOURCE_DIR}/mobo_openocd.cfg -c init -c halt)

add_custom_target(program
                  COMMAND ${OPENOCD_INIT} -c "flash banks" -c "flash list" -c "flash write_image ${CMAKE_CURRENT_BINARY_DIR}/mobo_mcu.elf" -c reset -c shutdown
                  DEPENDS mobo_mcu.elf)


add_custom_target(set_boot_vector
                  COMMAND ${OPENOCD_INIT} -c "at91sam3 gpnvm set 1" -c shutdown)

add_custom_target(query 
                  COMMAND ${OPENOCD_INIT} -c scan_chain -c targets sam3.cpu -c "at91sam3 info" -c reset -c shutdown)
 
