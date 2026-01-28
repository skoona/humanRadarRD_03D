set(COMPONENT_USED driver freertos esp_common log) 
idf_component_register(SRCS "humanRadarRD_03D.c"
                    REQUIRES ${COMPONENT_USED}
                    INCLUDE_DIRS "include")
