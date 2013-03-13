add_executable(rpalm-std.elf main.c startup.c comms.c imu.c io.c pins.c tactile.c)
target_link_libraries(rpalm-std.elf common_sam3s)
