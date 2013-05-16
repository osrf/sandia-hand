#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/palm_cli /dev/ttyUSB0 burn ../../firmware/build/rpalm/std/rpalm-std.bin
