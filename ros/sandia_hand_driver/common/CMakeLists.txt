rosbuild_add_library(sandia_hand hand.cpp finger.cpp motor_module.cpp serial_message_processor.cpp palm.cpp)
install(TARGETS sandia_hand DESTINATION lib)
