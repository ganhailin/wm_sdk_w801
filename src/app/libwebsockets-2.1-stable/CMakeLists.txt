get_filename_component(libname "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
file(GLOB_RECURSE ${libname}_src *.c)
set(remove_src_list
        minihuf.c
        minilex.c
        rewrite.c
        server.c
        server-handshake.c
        sha-1.c
        smtp.c
        ssl-http2.c
        ssl-server.c
        alloc.c
        daemonize.c
        extension.c
        extension-permessage-deflate.c
        getifaddrs.c
        hpack.c
        http2.c
        lejp.c
        lejp-conf.c
        libev.c
        libuv.c
        lws-plat-mbed3.c
        )
FOREACH(removefile ${remove_src_list})
    list(REMOVE_ITEM ${libname}_src "${CMAKE_CURRENT_SOURCE_DIR}/${removefile}")
ENDFOREACH()

add_library(${libname} ${${libname}_src})
include_directories(include)
message(addedlib:${libname})