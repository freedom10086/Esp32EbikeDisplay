/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <math.h>
#include <esp_event.h>
#include "lvgl.h"
#include "spl06.h"
#include "ms5611.h"
#include "esp_log.h"

static const char *TAG = "main_page";

#ifndef PI
#define PI  (3.14159f)
#endif

// LVGL image declare
LV_IMG_DECLARE(esp_logo)
LV_IMG_DECLARE(esp_text)

static lv_obj_t *chart;
static lv_coord_t chart_data[150] = {0};
static lv_coord_t chart_data2[150] = {0};
static uint16_t chart_data_len = 0;
static uint16_t chart_data2_len = 0;

static lv_chart_series_t * ser;
static lv_chart_series_t * ser2;

static lv_obj_t *lable1;
static lv_obj_t *lable2;

static void
pressure_sensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,
                              void *event_data) {
    pressure_sensor_data_t *data = NULL;
    switch (event_id) {
        case MS5611_SENSOR_UPDATE:
            data = (pressure_sensor_data_t *) event_data;
            ESP_LOGI(TAG, "ms5611 pressure: %.2f,temp: %.2f, altitude: %.2f", data->pressure, data->temp, data->altitude);
            if (chart_data_len == 0) {
                chart_data[chart_data_len] = data->altitude * 20;
            } else {
                chart_data[chart_data_len] = data->altitude * 20 - chart_data[0];
            }
            chart_data_len ++;
            if (chart_data_len == 150) {
                chart_data_len = 0;
            }

            static char buff[20];
            sprintf(buff, "%.2f", data->altitude);
            lv_label_set_text(lable1, buff);
            lv_chart_set_ext_y_array(chart, ser, (lv_coord_t *)chart_data);
            break;
        case SPL06_SENSOR_UPDATE:
            data = (pressure_sensor_data_t *) event_data;
            ESP_LOGI(TAG, "spl06 pressure: %.2f,temp: %.2f, altitude: %.2f", data->pressure, data->temp, data->altitude);
            if (chart_data2_len == 0) {
                chart_data2[chart_data2_len] = data->altitude * 20;
            } else {
                chart_data2[chart_data2_len] = data->altitude * 20 - chart_data2[0];
            }
            chart_data2_len ++;
            if (chart_data2_len == 150) {
                chart_data2_len = 0;
            }
            static char buff2[20];
            sprintf(buff2, "%.2f", data->altitude);
            lv_label_set_text(lable2, buff2);
            lv_chart_set_ext_y_array(chart, ser2, (lv_coord_t *)chart_data2);
            break;
        default:
            break;
    }
}

void sensor_init() {
    esp_event_loop_create_default();
    // init spl06
    spl06_init(NULL);
    // init ms5611
    //ms5611_init();
    esp_event_handler_register(BIKE_PRESSURE_SENSOR_MS5611_EVENT, ESP_EVENT_ANY_ID,
                               pressure_sensor_event_handler, NULL);
    esp_event_handler_register(BIKE_PRESSURE_SENSOR_EVENT, ESP_EVENT_ANY_ID,
                               pressure_sensor_event_handler, NULL);
}

static void slider_x_event_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    int32_t v = lv_slider_get_value(obj);
    lv_chart_set_zoom_x(chart, v);
}

static void slider_y_event_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    int32_t v = lv_slider_get_value(obj);
    lv_chart_set_zoom_y(chart, v);
}

/**
 * Display 1000 data points with zooming and scrolling.
 * See how the chart changes drawing mode (draw only vertical lines) when
 * the points get too crowded.
 */
void lv_example_chart_5(void)
{

}

void example_lvgl_demo_ui(lv_disp_t *disp) {
    sensor_init();

    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    /*Create a chart*/
    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_size(chart, 240, 220);
    lv_obj_align(chart, LV_ALIGN_TOP_MID, 0, 0);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -20, 60);

    /*Do not display points on the data*/
    lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);

    // data1
    ser = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    uint32_t pcnt = sizeof(chart_data) / sizeof(chart_data[0]);
    lv_chart_set_point_count(chart, pcnt);
    lv_chart_set_ext_y_array(chart, ser, (lv_coord_t *)chart_data);

    // data2
    ser2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    pcnt = sizeof(chart_data2) / sizeof(chart_data2[0]);
    lv_chart_set_point_count(chart, pcnt);
    lv_chart_set_ext_y_array(chart, ser2, (lv_coord_t *)chart_data2);

    // text
    lable1 = lv_label_create(scr);
    static lv_style_t style;
    lv_style_set_text_color(&style, lv_palette_main(LV_PALETTE_RED));
    lv_obj_add_style(lable1, &style, 0);
    lv_label_set_text(lable1, "000");
    lv_obj_align(lable1, LV_ALIGN_BOTTOM_LEFT, 5, -40);

    lable2 = lv_label_create(scr);
    static lv_style_t style2;
    lv_style_set_text_color(&style2, lv_palette_main(LV_PALETTE_BLUE));
    lv_obj_add_style(lable2, &style2, 0);
    lv_label_set_text(lable2, "000");
    lv_obj_align(lable2, LV_ALIGN_BOTTOM_RIGHT, -5, -40);
}
