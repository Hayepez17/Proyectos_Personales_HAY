idf_component_register(SRCS "uarts.c" "tareas.c" "modbus.c" "crc16.c" "init_uart.c" "main.c"
                    INCLUDE_DIRS ".")
list(APPEND EXTRA_COMPONENT_DIRS components)