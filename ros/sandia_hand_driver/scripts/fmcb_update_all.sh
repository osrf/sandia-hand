#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 mmburn_all ../../firmware/build/fmcb/std/fmcb-std.bin
