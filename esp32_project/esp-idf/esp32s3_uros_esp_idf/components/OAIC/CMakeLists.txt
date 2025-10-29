idf_component_register(SRCS "OAIC.c" 
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_timer)

# set(COMPONENT_REQUIRES "esp_system" "log" "freertos")
