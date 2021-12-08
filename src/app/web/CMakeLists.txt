get_filename_component(libname "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
file(GLOB_RECURSE ${libname}_src *.c)
set(remove_src_list
        fsdata_lwip.c
        fsdata_lwip_basic.c
        fsdata_lwip_ruigong.c
        fsdata_lwip_russian.c
        )
FOREACH(removefile ${remove_src_list})
    list(REMOVE_ITEM ${libname}_src "${CMAKE_CURRENT_SOURCE_DIR}/${removefile}")
ENDFOREACH()

add_library(${libname} ${${libname}_src})
include_directories(include)
message(addedlib:${libname})