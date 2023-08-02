set(srcs "main.c"
	     "connect_wifi.c" "init_pwm.c" "ultrasonic.c")
idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)
list(APPEND EXTRA_COMPONENT_DIRS components)