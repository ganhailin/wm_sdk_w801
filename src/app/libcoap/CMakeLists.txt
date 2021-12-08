get_filename_component(libname "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
file(GLOB_RECURSE ${libname}_src *.c)
set(remove_src_list
        address.c
        async.c
        block.c
        coap_io.c
        coap_io_lwip.c
        coap_time.c
        debug.c
        encode.c
        hashkey.c
        mem_libcoap.c
        net.c
        option.c
        pdu.c
        resource.c
        sr.c
        subscribe.c
        uri_libcoap.c
        )
FOREACH(removefile ${remove_src_list})
    list(REMOVE_ITEM ${libname}_src "${CMAKE_CURRENT_SOURCE_DIR}/${removefile}")
ENDFOREACH()

add_library(${libname} ${${libname}_src})
include_directories(include)
message(addedlib:${libname})