#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mflash_burn_fpga mobo.bin
