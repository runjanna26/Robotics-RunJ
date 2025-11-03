idf_component_register(SRCS "main.c" "config.h" "micro_ros_setup.h"
                    PRIV_REQUIRES spi_flash
                    PRIV_REQUIRES micro_ros_espidf_component
                    
                    INCLUDE_DIRS "."

                    REQUIRES watchdog_timer
                    REQUIRES AS5X47
                    REQUIRES AKMotors
                    REQUIRES OAIC
                    REQUIRES CPGRBF

                    REQUIRES DynamixelSDK-c
                    )