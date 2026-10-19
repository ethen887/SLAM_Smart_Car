add_library(bsp INTERFACE)

target_sources(bsp
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/robot_config.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Can/bsp_can.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Can/bsp_can.h
        ${CMAKE_CURRENT_SOURCE_DIR}/I2c/bsp_iic.c
        ${CMAKE_CURRENT_SOURCE_DIR}/I2c/bsp_iic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Log/bsp_log.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Log/bsp_log.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PWM/bsp_pwm.c
        ${CMAKE_CURRENT_SOURCE_DIR}/PWM/bsp_pwm.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Uart/bsp_uart.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Uart/bsp_uart.h
)

target_include_directories(bsp
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Can
        ${CMAKE_CURRENT_SOURCE_DIR}/I2c
        ${CMAKE_CURRENT_SOURCE_DIR}/Log
        ${CMAKE_CURRENT_SOURCE_DIR}/PWM
        ${CMAKE_CURRENT_SOURCE_DIR}/Uart
)
