add_subdirectory(sandia_hand_description)

ExternalProject_Add(sandia_hand_teleop
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sandia_hand_teleop
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_teleop
                    CMAKE_ARGS
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DSANDIA_HAND_VERSION:STRING=${SANDIA_HAND_VERSION}
                    INSTALL_COMMAND "")
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_teleop install)")

ExternalProject_Add(sandia_hand_driver
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sandia_hand_driver
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_driver
                    CMAKE_ARGS
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DSANDIA_HAND_VERSION:STRING=${SANDIA_HAND_VERSION}
                    INSTALL_COMMAND "")
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_driver install)")

ExternalProject_Add(sandia_hand_msgs
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sandia_hand_msgs
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_msgs
                    CMAKE_ARGS
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DSANDIA_HAND_VERSION:STRING=${SANDIA_HAND_VERSION}
                    INSTALL_COMMAND "")
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/sandia_hand_msgs install)")

