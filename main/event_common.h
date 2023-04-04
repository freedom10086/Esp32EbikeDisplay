#ifndef EVENT_COMMON_H
#define EVENT_COMMON_H

typedef enum {
    SPL06_SENSOR_UPDATE,
    MS5611_SENSOR_UPDATE,
} spl06_event_id_t;

typedef struct {
    float temp;

    float pressure;

    float altitude;

} pressure_sensor_data_t;

#endif // EVENT_COMMON_H