cmake_minimum_required(VERSION 3.20)

set(DOWNLOAD_PORT ttyUSB0)
set(DOWNLOAD_SPEED 2M)

FILE (WRITE ${CMAKE_BINARY_DIR}/timestamp.cmake "STRING(TIMESTAMP TIMEZ UTC)\n")
FILE (APPEND ${CMAKE_BINARY_DIR}/timestamp.cmake "FILE(WRITE timestamp.h \"#ifndef TIMESTAMP_H\\n\")\n")
FILE (APPEND ${CMAKE_BINARY_DIR}/timestamp.cmake "FILE(APPEND timestamp.h \"#define TIMESTAMP_H\\n\\n\")\n")
FILE (APPEND ${CMAKE_BINARY_DIR}/timestamp.cmake "FILE(APPEND timestamp.h \"#define _TIMEZ_ \\\"\${TIMEZ}\\\"\\n\\n\")\n")
FILE (APPEND ${CMAKE_BINARY_DIR}/timestamp.cmake "FILE(APPEND timestamp.h \"#endif // TIMESTAMP_H\\n\")\n")
ADD_CUSTOM_TARGET (
        timestamp
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/timestamp.cmake
        ADD_DEPENDENCIES ${CMAKE_BINARY_DIR}/timestamp.cmake)

include(ExternalProject)
#message("${CMAKE_CURRENT_BINARY_DIR}")
ExternalProject_Add (wm_tools
        DOWNLOAD_COMMAND "" # Disable download step
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools/w800" # Source dir for configuration
        INSTALL_COMMAND cp wm_tool ${CMAKE_CURRENT_BINARY_DIR}
        )

set(CMAKE_TOOLCHAIN_FILE "csky-abiv2-toolchain.cmake")
set(output_name w806-demo)
project(w806 C CXX ASM)

execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -mcpu=ck804ef  -mlittle-endian  -print-file-name=crti.o
        OUTPUT_VARIABLE crti_o_path
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -mcpu=ck804ef  -mlittle-endian  -print-file-name=crtbegin.o
        OUTPUT_VARIABLE crtbegin_o_path
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -mcpu=ck804ef  -mlittle-endian  -print-file-name=crtend.o
        OUTPUT_VARIABLE crtend_o_path
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -mcpu=ck804ef  -mlittle-endian  -print-file-name=crtn.o
        OUTPUT_VARIABLE crtn_o_path
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
set (extra_input_file ${crti_o_path} ${crtbegin_o_path} ${crtend_o_path} ${crtn_o_path})
#set (CMAKE_CXX_STANDARD 17)
#set (CMAKE_C_STANDARD 11)
set(include_dir
        include
        include/app
        include/arch/xt804
        include/arch/xt804/csi_core
        include/arch/xt804/csi_dsp
        include/driver
        include/net
        include/os
        include/platform
        include/wifi
        include/bt
        platform/common/params
        platform/inc
        platform/sys
        src/app/wm_atcmd
        src/app/dhcpserver
        src/app/dnsserver
        src/app/web
        src/app/cloud
        src/app/cJSON
        src/app/rmms
        src/app/ntp
        src/app/httpclient
        src/app/oneshotconfig
        src/app/iperf
        src/app/mqtt
        src/app/ping
        src/app/polarssl/include
        src/app/mDNS/mDNSPosix
        src/app/mDNS/mDNSCore
        src/app/ota
        src/app/libwebsockets-2.1-stable
        src/app/fatfs
        src/app/mbedtls/include
        src/app/mbedtls/ports
        src/network/api2.0.3
        src/network/lwip2.0.3/include
        src/os/rtos/include
        src/app/factorycmd
        demo
        src/bt/blehost/ext/tinycrypt/include
        src/bt/blehost/nimble/host/include
        src/bt/blehost/nimble/host/mesh/include
        src/bt/blehost/nimble/host/porting/w800/include
        src/bt/blehost/nimble/host/services/gap/include
        src/bt/blehost/nimble/host/services/gatt/include
        src/bt/blehost/nimble/host/services/bas/include
        src/bt/blehost/nimble/host/services/dis/include
        src/bt/blehost/nimble/host/store/config/include
        src/bt/blehost/nimble/host/store/ram/include
        src/bt/blehost/nimble/host/util/include
        src/bt/blehost/nimble/include
        src/bt/blehost/nimble/transport/uart/include
        src/bt/blehost/porting/w800/include
        src/bt/host
        src/bt/host/main
        src/bt/host/include
        src/bt/host/stack
        src/bt/host/stack/include
        src/bt/host/gki/wm
        src/bt/host/gki/common
        src/bt/host/vnd/include
        src/bt/host/bta/include
        src/bt/host/osi/include
        src/bt/host/hci/include
        src/bt/host/btif/include
        src/bt/host/embdrv/sbc/encoder/include
        src/bt/host/embdrv/sbc/decoder/include
        src/bt/host/btcore/include
        )

set(extflags "")

set(compile_flags
        -mcpu=ck804ef
        -mhard-float
        -DGCC_COMPILE=1
        -DTLS_CONFIG_CPU_XT804=1
        -DNIMBLE_FTR=0
        -O3
        -Os
#        -g3
        -Wall
        -ffunction-sections
        -fdata-sections
        -finput-charset=UTF-8
        -fextended-identifiers
        #        -Wa,--gdwarf2
        )
add_compile_options(${compile_flags})

include_directories(${include_dir} ${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(app)
add_subdirectory(demo)
add_subdirectory(src/app)
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(sysapps ${CMAKE_CURRENT_SOURCE_DIR}/src/app)


add_subdirectory(src/os)
add_subdirectory(src/bt)
add_subdirectory(src/network)
add_subdirectory(platform)

set(dummy_src dummy.cpp)
add_executable(${output_name}.elf ${dummy_src})



set(compoment_list
        app
        demo
        platform
        rtos
        network
        bthost
        ${sysapps}
        )
add_dependencies(${output_name}.elf wm_tools ${compoment_list} timestamp)

target_link_directories(${output_name}.elf PRIVATE lib/w800)
target_link_options(${output_name}.elf PUBLIC
        ${extra_input_file}
        -mcpu=ck804ef
        -nostartfiles
        -Wl,--gc-sections
        -Wl,-zmax-page-size=1024
        -mhard-float
        -Wl,-Map,${output_name}.map
        -Wl,--ckmap=${output_name}_ckmap.map
        -Wl,-T${CMAKE_CURRENT_SOURCE_DIR}/ld/w800/gcc_csky.ld
        )
target_link_libraries(${output_name}.elf
        ${compoment_list}
        dsp wlan btcontroller
        )

add_custom_command(TARGET ${output_name}.elf
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${output_name}.elf ${output_name}.bin
        COMMAND ${CMAKE_OBJDUMP} ARGS -S ${output_name}.elf  > W806.asm
        COMMAND mkdir ARGS -p ${CMAKE_CURRENT_BINARY_DIR}/w800
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/wm_tool ARGS -b ${output_name}.bin -it 1 -fc 0 -ra 80d0400 -ih 80d0000 -ua 8010000 -nh 0  -un 0 -o ${output_name}
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/wm_tool ARGS -b ${CMAKE_CURRENT_SOURCE_DIR}/tools/w800/w800_secboot.bin -it 0 -fc 0 -ra 8002400 -ih 8002000 -ua 8010000 -nh 80d0000  -un 0 -o ${output_name}_secboot
        COMMAND cat ARGS ${output_name}_secboot.img ${output_name}.img > ${output_name}.fls
        )

add_custom_target(flash
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/wm_tool -c ${DOWNLOAD_PORT} -rs at -ds ${DOWNLOAD_SPEED} -dl ${output_name}.fls
        )
add_dependencies(flash ${output_name}.elf)