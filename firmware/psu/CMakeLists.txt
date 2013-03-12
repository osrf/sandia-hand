set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_FLAGS "-Os -std=c99 -g -DF_CPU=1000000 -Wall -mmcu=atmega328p")
set(CMAKE_C_LINK_FLAGS "-Wl,-Map,psu.map")
add_executable(psu main.c)
