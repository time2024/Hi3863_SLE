#===============================================================================
# @brief    cmake file
# Copyright (c) CompanyNameMagicTag 2023-2023. All rights reserved.
#===============================================================================
if(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/epaper.c
    ${CMAKE_CURRENT_SOURCE_DIR}/GUI/GUI_Paint.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Fonts/font8.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Fonts/font12.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Fonts/font16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Fonts/font20.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Fonts/font24.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server_adv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart.c
)
set(HEADER_LIST 
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server
    ${CMAKE_CURRENT_SOURCE_DIR}
    )
elseif(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client/sle_uart_client.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart.c
)
set(HEADER_LIST ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client)
endif()

set(SOURCES "${SOURCES}" ${SOURCES_LIST} PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" ${HEADER_LIST} PARENT_SCOPE)