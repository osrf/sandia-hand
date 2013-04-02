#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mmcu_burn ../../firmware/build/mobo/mcu/mobo-mcu.bin
