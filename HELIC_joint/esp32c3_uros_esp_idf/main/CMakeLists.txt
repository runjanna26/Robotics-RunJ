idf_component_register(SRCS "main.c" "config.h" "micro_ros_setup.h"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component

                    REQUIRES watchdog_timer
                    )

target_link_libraries(${COMPONENT_LIB} INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/micro_ros_espidf_component/libmicroros.a
)