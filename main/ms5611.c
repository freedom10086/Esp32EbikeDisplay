#include <math.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "esp_event.h"

#include "ms5611.h"
#include "event_common.h"

ESP_EVENT_DEFINE_BASE(BIKE_PRESSURE_SENSOR_MS5611_EVENT);

#define MS5611_ADDR 0x77

#define MSE5611_Reset 0x1E
#define Convert_D1_256 0x40
#define Convert_D1_512 0x42
#define Convert_D1_1024 0x44
#define Convert_D1_2048 0x46
#define Convert_D1_4096 0x48

#define Convert_D2_256 0x50
#define Convert_D2_512 0x52
#define Convert_D2_1024 0x54
#define Convert_D2_2048 0x56
#define Convert_D2_4096 0x58
#define ADC_Read 0x00

#define PROM_Read_0 0xA0
#define PROM_Read_1 0xA2
#define PROM_Read_2 0xA4
#define PROM_Read_3 0xA6
#define PROM_Read_4 0xA8
#define PROM_Read_5 0xAA
#define PROM_Read_6 0xAC
#define PROM_Read_7 0xAE

#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 20000

#define I2C_MASTER_SDA_IO 1
#define I2C_MASTER_SCL_IO 0

#define I2C_MASTER_TIMEOUT_MS 10

#define TAG "MS5611"

uint16_t _Reserve = 0;
uint16_t _C[6] = {0};
uint32_t _D[2] = {0};

int32_t _Temp_x100 = 0;
int32_t _P = 0;

static pressure_sensor_data_t data;

/**
 * ANALOG DIGITAL CONVERTER (ADC)
 * OSR      Min.    Typ     Max.
 * 4096     7.40    8.22    9.04
 * 2048     3.72    4.13    4.54
 * 1024     1.88    2.08    2.28
 * 512      0.95    1.06    1.17
 * 256      0.48    0.54    0.60
 */

static void ms5611_task_entry(void *arg);

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t i2c_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MS5611_ADDR,
                                                 &reg_addr, 1, data, len,
                                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read i2c failed, for addr %x,  %s", reg_addr, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t i2c_write_cmd(uint8_t reg_addr) {
    int ret;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MS5611_ADDR,
                                     &reg_addr, 1,
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write i2c cmd failed, for addr %x,  %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t i2c_write(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MS5611_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

void ms5611_init() {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,         // select GPIO specific to your project
            //.sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,         // select GPIO specific to your project
            //.scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
            // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);

    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "ms5611 I2C initialized successfully sda: %d, scl:%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    ms5611_reset();

    uint16_t ms5611_CRC = 0;
    // load config from 0xA0 - 0xAE
    for (int i = 0; i < 8; ++i) {
        uint8_t buff[2] = {0};
        i2c_read(PROM_Read_0 + i * 2, buff, 2);
        uint16_t value = (buff[0] << 8) + buff[1];
        if (i == 0) {
            _Reserve = value;
        } else if (i == 7) {
            ms5611_CRC = value;
        } else {
            _C[i -1] = value;
        }
    }
    ESP_LOGI(TAG, "ms5611 read reserve:%x, c1:%x, c2:%x, c3:%x, c4:%x, c5:%x, c6:%x crc:%x",
             _Reserve, _C[0], _C[1], _C[2], _C[3], _C[4], _C[5], ms5611_CRC);

    // TODO check crc

    /* Create NMEA Parser task */
    BaseType_t err = xTaskCreate(
            ms5611_task_entry,
            "ms5611",
            2048,
            NULL,
            5,
            NULL);
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "create ms5611 task failed");
        return;
    }
}

void ms5611_reset() {
    i2c_write_cmd(MSE5611_Reset);
    vTaskDelay(pdMS_TO_TICKS(16));
    ESP_LOGI(TAG, "ms5611 reset");
}

void ms5611_read_pressure_pre() {
    i2c_write_cmd(Convert_D1_4096);
    //ESP_LOGI(TAG, "ms5611 read pressure pre");
}

void ms5611_read_pressure() {
    uint8_t buff[3]= {0};
    i2c_read(ADC_Read, buff, 3);
    _D[0] = (buff[0] << 16) + (buff[1] << 8) + buff[2];
    ESP_LOGI(TAG, "ms5611 read pressure, raw:%lx", _D[0]);
}

void ms5611_read_temp_pre() {
    i2c_write_cmd(Convert_D2_4096);
    //ESP_LOGI(TAG, "ms5611 read temp pre");
}

void ms5611_read_temp() {
    uint8_t buff[3]= {0};
    i2c_read(ADC_Read, buff, 3);
    _D[1] = (buff[0] << 16) + (buff[1] << 8) + buff[2];
    // ESP_LOGI(TAG, "ms5611 read temp, raw:%lx", _D[1]);
}

void ms5611_get_pressure() {
    int64_t OFF = 0, OFF2 = 0, SENS = 0, SENS2 = 0, T2 = 0;
    int64_t dT = 0;
    dT = _D[1] - (((int64_t) _C[4]) << 8);
    _Temp_x100 = 2000 + (((int64_t) dT * _C[5]) >> 23);

    // ESP_LOGI(TAG, "ms5611 temp is %.2f", _Temp_x100 / 100.0f);

    if (_Temp_x100 < 2000) {
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * (_Temp_x100 - 2000) * (_Temp_x100 - 2000) >> 1;
        SENS2 = 5 * (_Temp_x100 - 2000) * (_Temp_x100 - 2000) >> 2;
    }
    if (_Temp_x100 < -1500) {
        OFF2 = OFF2 + 7 * (_Temp_x100 + 1500) * (_Temp_x100 + 1500);
        SENS2 = SENS2 + (11 * (_Temp_x100 + 1500) * (_Temp_x100 + 1500) >> 1);
    }
    OFF = (((int64_t) _C[1]) << 16) + ((_C[3] * dT) >> 7);
    SENS = (((int64_t) _C[0]) << 15) + ((_C[2] * dT) >> 8);
    OFF -= OFF2;
    _Temp_x100 -= T2;
    SENS -= SENS2;
    _P = (((_D[0] * SENS) >> 21) - OFF) >> 15;
}

static void ms5611_task_entry(void *arg) {
    // skip first result
    ms5611_read_pressure_pre();
    vTaskDelay(pdMS_TO_TICKS(25));
    ms5611_read_pressure();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(300));

        ms5611_read_temp_pre();
        vTaskDelay(pdMS_TO_TICKS(20));
        ms5611_read_temp();

        ms5611_read_pressure_pre();
        vTaskDelay(pdMS_TO_TICKS(25));
        ms5611_read_pressure();

        ms5611_get_pressure();
        float height = calc_altitude(_P);
        ESP_LOGI(TAG, "ms5611 height: %.2f, temp:%.2f", height, _Temp_x100 * 1.0f / 100);

        data.altitude = height;
        data.pressure = _P;
        data.temp = _Temp_x100 * 1.0f / 100;

        esp_event_post(BIKE_PRESSURE_SENSOR_MS5611_EVENT, MS5611_SENSOR_UPDATE,
                       &data, sizeof(pressure_sensor_data_t), 100 / portTICK_PERIOD_MS);
    }
}