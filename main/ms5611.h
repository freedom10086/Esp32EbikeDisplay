#ifndef MS5611
#define MS5611

#include "pressure_common.h"

ESP_EVENT_DECLARE_BASE(BIKE_PRESSURE_SENSOR_MS5611_EVENT);

void ms5611_init();
void ms5611_reset();
void ms5611_read_pressure_pre(void);
void ms5611_read_pressure(void);
void ms5611_read_temp_pre(void);
void ms5611_read_temp(void);
void ms5611_get_pressure(void);

#endif
