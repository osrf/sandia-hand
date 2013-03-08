macro(sam3s_create_apps board apps)
  include_directories(${PROJECT_SOURCE_DIR}/common_sam3s/include)
  add_definitions(-Dsam3s4)
  set(CMAKE_C_LINK_FLAGS ${SAM3S_APP_LINK_FLAGS})
  message("creating apps for ${board}")
  foreach (app ${apps})
    message("creating app ${app} for board ${board}")
    add_subdirectory(${app})
    add_custom_target(${board}-${app}-disassemble
                      COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objdump -S -d ${CMAKE_CURRENT_BINARY_DIR}/${app}/${board}-${app}.elf | vim -
                      DEPENDS ${board}-${app}.elf)
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${app}/${board}-${app}.bin
                       COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objcopy -O binary ${app}/${board}-${app}.elf ${app}/${board}-${app}.bin
                       DEPENDS ${board}-${app}.elf)
    add_custom_target(do_${board}-${app}.bin ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${app}/${board}-${app}.bin)
  endforeach()
endmacro()
