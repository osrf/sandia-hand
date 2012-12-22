add_subdirectory(sandia_hand_description)

ExternalProject_Add(sandia_hand_teleop
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sandia_hand_teleop
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_teleop
                    CMAKE_ARGS
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                    INSTALL_COMMAND "")
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_teleop install)")

