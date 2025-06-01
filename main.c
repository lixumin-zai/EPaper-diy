#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <epdiy.h>
#include "sdkconfig.h"
#include "firasans_12.h"
#include "firasans_20.h"

// 添加蓝牙相关头文件
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gatts_api.h>
#include <esp_system.h>
// 蓝牙相关定义
#define GATTS_SERVICE_UUID_IMAGE   0x00FF
#define GATTS_CHAR_UUID_IMAGE_DATA 0xFF01
#define GATTS_NUM_HANDLE_IMAGE     4

#define DEVICE_NAME "ESP32-EPaper"
#define MANUFACTURER_DATA_LEN  4

// 图像缓冲区定义
#define MAX_IMAGE_SIZE (1448 * 1072 / 2) // 根据img_board.h中的图像大小定义
static uint8_t received_image_data[MAX_IMAGE_SIZE];
static uint32_t received_image_width = 1448;
static uint32_t received_image_height = 1072;
static uint32_t received_data_length = 0;
static bool image_received_complete = false;

// 蓝牙服务和特征句柄
static uint16_t image_handle_table[GATTS_NUM_HANDLE_IMAGE];
static esp_gatt_if_t image_gatts_if;

#define WAVEFORM EPD_BUILTIN_WAVEFORM

#define epd_poweron() digitalWrite(46,1)
#define epd_poweroff() digitalWrite(46,0)



// choose the default demo board depending on the architecture
#ifdef CONFIG_IDF_TARGET_ESP32
#define DEMO_BOARD epd_board_v6
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define DEMO_BOARD epd_board_v7
#endif

EpdiyHighlevelState hl;

const EpdDisplay_t ED060KD1 = {
    .width = 1448,
    .height = 1072,
    .bus_width = 8,
    .bus_speed = 20,
    .default_waveform = &epdiy_ED060SCT,
    .display_type = DISPLAY_TYPE_GENERIC,
};

// 显示调试信息的通用函数
static void display_debug_info(const char* message, bool clear_screen) {
    if (clear_screen) {
        epd_poweron();
        epd_clear();
        epd_poweroff();
        return;
    }

    uint8_t* fb = epd_hl_get_framebuffer(&hl);
    const EpdFont* font;
    if (epd_width() < 1000) {
        font = &FiraSans_12;
    } else {
        font = &FiraSans_20;
    }
    int temperature = epd_ambient_temperature();

    int cursor_x = epd_rotated_display_width() / 2;
    int cursor_y = epd_rotated_display_height() / 2;

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_CENTER;

    epd_hl_set_all_white(&hl);
    delay(1000);
    epd_write_string(font, message, &cursor_x, &cursor_y, fb, &font_props);
    
    epd_poweron();
    epd_hl_update_screen(&hl, MODE_GL16, temperature);
    epd_poweroff();
}

// 蓝牙广播数据
static uint8_t manufacturer_data[MANUFACTURER_DATA_LEN] = {0x12, 0x34, 0x56, 0x78};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = manufacturer_data,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// 蓝牙广播参数
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT服务回调函数
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// 服务定义
static struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} image_profile_tab = {
    .gatts_cb = gatts_profile_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
};

// 蓝牙事件回调函数
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            char error_msg[64];
            sprintf(error_msg, "广播启动失败: %d", param->adv_start_cmpl.status);
        } 
        // else {
            
        // }
        break;
    default:
        break;
    }
}

void idf_setup() {
    epd_init(&DEMO_BOARD, &ED060KD1, EPD_LUT_64K);

    epd_set_vcom(1560);

    hl = epd_hl_init(WAVEFORM);

    epd_set_rotation(EPD_ROT_LANDSCAPE);

    printf(
        "Dimensions after rotation, width: %d height: %d\n\n", epd_rotated_display_width(),
        epd_rotated_display_height()
    );

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
}

void idf_loop() {
    display_debug_info("", true);
    display_debug_info("hello world", false);

    display_debug_info("lismin", false);
    
    epd_deinit();
    esp_deep_sleep_start();
}

#ifndef ARDUINO_ARCH_ESP32
void app_main() {
    idf_setup();
    idf_loop();
}
#endif