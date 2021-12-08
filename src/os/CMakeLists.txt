file(GLOB_RECURSE rtos_src *.c *.S)
add_library(rtos ${rtos_src})