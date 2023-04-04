file(GLOB_RECURSE IMAGE_SOURCES images/*.c)

idf_component_register(SRCS "main.c"
        "pressure_sensor_ui.c"
        ${IMAGE_SOURCES}
        "pressure_common.c" "spl06.c" "ms5611.c"
        INCLUDE_DIRS ".")
