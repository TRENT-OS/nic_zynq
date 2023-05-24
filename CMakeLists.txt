#
# Zynq NIC driver
#
# Copyright (C) 2023, HENSOLDT Cyber GmbH
#
# SPDX-License-Identifier: BSD-3-Clause
#

cmake_minimum_required(VERSION 3.17)


if(SDK_USE_CAMKES)

    os_sdk_import_from_global_components(
        "components/modules/fdt-bind-driver"
        "components/modules/single-threaded"
    )

endif()  # SDK_USE_CAMKES


#-------------------------------------------------------------------------------
function(NIC_ZYNQ_DeclareCAmkESComponent name)

    cmake_parse_arguments(
        PARSE_ARGV
        1
        NIC_ZYNQ
        "" # Option arguments
        "" # Single arguments
        "C_FLAGS" # Multiple arguments
    )

    DeclareCAmkESComponent(
        ${name}
        SOURCES
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/src/ethdriver.c
        INCLUDES
            include
        C_FLAGS
            ${NIC_ZYNQ_C_FLAGS}
        LIBS
            os_core_api
            ethdrivers
    )

    if(EXISTS ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/src/plat/${KernelPlatform}/plat.c)
        DeclareCAmkESComponent(${name} SOURCES ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/src/plat/${KernelPlatform}/plat.c)
    endif()

endfunction()

#-------------------------------------------------------------------------------
# declare CAmkES components for all NICs of a platform
function(NIC_ZYNQ_DeclareCAmkESComponents_for_NICs)

    set(dir_plat "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/plat/${PLATFORM}")

    if(NOT IS_DIRECTORY "${dir_plat}")
        message(FATAL_ERROR "unsupported platform: '${PLATFORM}'")
    endif()

    CAmkESAddCPPInclude("${dir_plat}")
    include("${dir_plat}/plat_nic_zynq.cmake")

endfunction()
