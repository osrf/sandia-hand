rosbuild_add_library(sandia_hand sandia_hand.cpp finger.cpp motor_module.cpp message_processor.cpp)
install(TARGETS sandia_hand DESTINATION lib)
