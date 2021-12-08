file(GLOB_RECURSE platform_src *.c )
add_library(platform ${platform_src} arch/xt804/bsp/vectors.S arch/xt804/bsp/startup.S)