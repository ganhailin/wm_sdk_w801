file(GLOB_RECURSE network_src lwip2.0.3/api/*.c lwip2.0.3/netif/*.c lwip2.0.3/core/*.c api2.0.3/*.c)
file(GLOB network_src_lwip lwip2.0.3/*.c)
set(remove_src_list
        lwip2.0.3/api/pppapi.c
        lwip2.0.3/core/timers.c
        lwip2.0.3/core/ipv4/ip_frag.c
        )
FOREACH(removefile ${remove_src_list})
    list(REMOVE_ITEM network_src "${CMAKE_CURRENT_SOURCE_DIR}/${removefile}")
ENDFOREACH()
add_library(network ${network_src} ${network_src_lwip})