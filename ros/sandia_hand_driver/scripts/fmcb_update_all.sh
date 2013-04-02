#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mmburn_all ../../firmware/build/fmcb/std/fmcb-std.bin
