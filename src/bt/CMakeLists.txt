file(GLOB_RECURSE bthost_src *.c *.S)
add_library(bthost ${bthost_src})