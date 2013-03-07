set(CMAKE_C_LINK_FLAGS "-g -mthumb -mcpu=cortex-m3 -T\"${CMAKE_CURRENT_SOURCE_DIR}/bl_sam3s.ld\" -Wl,-Map=memory.map -Wl,--cref --entry=_bootloader_start -Wl,--no-gc-sections -Wl,--warn-common -Wl,--unresolved-symbols=report-all -Wl,--no-undefined -nostdlib")
add_definitions(-Dsam3s4)
include_directories(${PROJECT_SOURCE_DIR}/common_sam3s)
add_executable(bl_f2.elf bl_sam3s.c bl_sam3s_startup.S bl_f2.c)

#add_executable(mobo-mcu.elf main.c startup.c fpga_spi.c power.c enet.c finger.c cam.c)
#set_target_properties(mobo_mcu.elf PROPERTIES LINK_FLAGS "-T\"${CMAKE_CURRENT_SOURCE_DIR}/../common_sam3x/sam3x8e.ld\"")

