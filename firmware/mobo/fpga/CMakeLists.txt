add_custom_target(mobo-fpga-load
#                  COMMAND ${OPENOCD_INIT} -c "svf -tap spartan6.tap ${CMAKE_CURRENT_SOURCE_DIR}/mobo.svf " -c reset -c shutdown)
                  #COMMAND ${OPENOCD_INIT} -c "xsvf spartan6.tap ${CMAKE_CURRENT_SOURCE_DIR}/mobo.xsvf " -c reset -c shutdown)
                  COMMAND ${OPENOCD_INIT} -c "xsvf spartan6.tap ${CMAKE_CURRENT_SOURCE_DIR}/mobo.xsvf " -c reset -c shutdown)
                  #COMMAND . $ENV{ISEDIR}/settings64.sh && ${OPENOCD_INIT} -c xsvf spartan6 mobo.xsvf -c reset -c shutdown)

