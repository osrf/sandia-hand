set(CMAKE_C_LINK_FLAGS ${SAM3S_APP_LINK_FLAGS})
add_definitions(-Dsam3s4)

set(fmcb_apps minimal)
#set(fmcb_apps minimal std)
add_definitions(-Dsam3s4)
include_directories(${PROJECT_SOURCE_DIR}/common_sam3s/include)

foreach(app ${fmcb_apps})
  add_subdirectory(${app})
  add_custom_target(fmcb-${app}-disassemble
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objdump -S -d ${CMAKE_CURRENT_BINARY_DIR}/${app}/fmcb-${app}.elf | vim -
                    DEPENDS fmcb-${app}.elf)
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${app}/fmcb-${app}.bin
                     COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objcopy -O binary ${app}/fmcb-${app}.elf ${app}/fmcb-${app}.bin
                     DEPENDS fmcb-${app}.elf)
  add_custom_target(do_fmcb-${app}.bin ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${app}/fmcb-${app}.bin)
endforeach()
