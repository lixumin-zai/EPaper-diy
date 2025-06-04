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
#include <nvs.h>
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
#define IMAGE_BUFFER_SIZE (300 * 396) // 最大支持电子墨水屏分辨率大小的图片
static uint8_t image_buffer[IMAGE_BUFFER_SIZE];
static uint32_t image_buffer_index = 0;
static uint32_t image_width = 0;
static uint32_t image_height = 0;
static bool image_header_received = false;
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
    font = &FiraSans_20;
    int temperature = epd_ambient_temperature();

    int cursor_x = epd_rotated_display_width() / 2;
    int cursor_y = epd_rotated_display_height() / 2;

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_CENTER;


    epd_hl_set_all_white(&hl);

    epd_write_string(font, message, &cursor_x, &cursor_y, fb, &font_props);
    epd_poweron();
    epd_hl_update_screen(&hl, MODE_GL16, temperature);
    epd_poweroff();
    delay(3000);
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
        break;
    default:
        break;
    }
}


// 处理接收到的图像数据
static void process_received_image() {
    if (!image_received_complete || image_width == 0 || image_height == 0) {
        display_debug_info("data error", false);
        return;
    }
    
    display_debug_info("process...", false);
    
    // 获取帧缓冲区
    uint8_t* fb = epd_hl_get_framebuffer(&hl);
    
    // 清空帧缓冲区
    epd_hl_set_all_white(&hl);
    
    // 计算缩放比例（如果需要）
    float scale_x = (float)epd_rotated_display_width() / image_width;
    float scale_y = (float)epd_rotated_display_height() / image_height;
    float scale = scale_x < scale_y ? scale_x : scale_y; // 取较小的缩放比例
    
    // 计算居中显示的偏移量
    int offset_x = (epd_rotated_display_width() - (int)(image_width * scale)) / 2;
    int offset_y = (epd_rotated_display_height() - (int)(image_height * scale)) / 2;
    
    // 将图像数据复制到帧缓冲区（这里需要根据实际图像格式进行适配）
    // 假设图像数据是4位灰度图，每个像素占4位
    for (int y = 0; y < image_height; y++) {
        for (int x = 0; x < image_width; x += 2) { // 每个字节包含两个像素
            uint8_t pixel_pair = image_buffer[(y * image_width + x) / 2];
            uint8_t pixel1 = (pixel_pair >> 4) & 0x0F; // 高4位
            uint8_t pixel2 = pixel_pair & 0x0F;        // 低4位
            
            // 计算目标位置（考虑缩放和居中）
            int target_x1 = offset_x + (int)(x * scale);
            int target_x2 = offset_x + (int)((x + 1) * scale);
            int target_y = offset_y + (int)(y * scale);
            
            // 确保坐标在有效范围内
            if (target_x1 >= 0 && target_x1 < epd_rotated_display_width() && 
                target_y >= 0 && target_y < epd_rotated_display_height()) {
                // 将4位灰度值转换为8位灰度值
                uint8_t gray1 = (pixel1 << 4) | pixel1; // 扩展到8位
                epd_draw_pixel(target_x1, target_y, gray1, fb);
            }
            
            if (target_x2 >= 0 && target_x2 < epd_rotated_display_width() && 
                target_y >= 0 && target_y < epd_rotated_display_height()) {
                uint8_t gray2 = (pixel2 << 4) | pixel2; // 扩展到8位
                epd_draw_pixel(target_x2, target_y, gray2, fb);
            }
        }
    }
    
    // 更新显示
    int temperature = epd_ambient_temperature();
    epd_poweron();
    epd_hl_update_screen(&hl, MODE_GL16, temperature);
    epd_poweroff();
    
    // 重置状态，准备接收下一张图片
    image_header_received = false;
    image_received_complete = false;
    image_buffer_index = 0;
}

// GATT服务回调函数实现
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        {
   
            image_profile_tab.service_id.is_primary = true;
            image_profile_tab.service_id.id.inst_id = 0x00;
            image_profile_tab.service_id.id.uuid.len = ESP_UUID_LEN_16;
            image_profile_tab.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_IMAGE;

            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);

            esp_ble_gatts_create_service(gatts_if, &image_profile_tab.service_id, GATTS_NUM_HANDLE_IMAGE);
        }
        break;
    case ESP_GATTS_CREATE_EVT:
        {
            
            image_profile_tab.service_handle = param->create.service_handle;

            esp_bt_uuid_t char_uuid;
            char_uuid.len = ESP_UUID_LEN_16;
            char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_IMAGE_DATA;

            esp_ble_gatts_add_char(image_profile_tab.service_handle, &char_uuid,
                                  ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                  NULL, NULL);
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        {
            image_profile_tab.char_handle = param->add_char.attr_handle;
            
            // 启动服务
            esp_ble_gatts_start_service(image_profile_tab.service_handle);
        }
        break;
    case ESP_GATTS_START_EVT:
        {
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        {   
            image_profile_tab.conn_id = param->connect.conn_id;
            // 添加设备连接信息显示
            char connect_msg[64];
            sprintf(connect_msg, "device connect, connect id: %d", param->connect.conn_id);
            display_debug_info(connect_msg, false);
        }
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        {
            char reason_str[32] = "Unknown Reason";
            switch(param->disconnect.reason) {
                case 0x13: strcpy(reason_str, "User Terminated Connection"); break;
                case 0x16: strcpy(reason_str, "Connection Timeout"); break;
                case 0x22: strcpy(reason_str, "Remote Device Terminated"); break;
                case 0x08: strcpy(reason_str, "Supervision Timeout"); break;
                default: sprintf(reason_str, "Code: 0x%x", param->disconnect.reason); break;
            }
            char info_msg[64];
            sprintf(info_msg, "disconnect: %s", reason_str);
            display_debug_info(info_msg, false);
            // 重新开始广播
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == image_profile_tab.char_handle) {
            // 处理接收到的图像数据
            display_debug_info("write", false);
            if (param->write.len > 0) {
                char info_msg[64];
                
                // 第一个数据包包含图像头信息（宽度和高度）
                if (!image_header_received && param->write.len >= 8) {
                    // 提取图像宽度和高度信息（前8个字节）
                    image_width = *(uint32_t*)param->write.value;
                    image_height = *(uint32_t*)(param->write.value + 4);
                    
                    // 显示图像信息
                    sprintf(info_msg, "recive: %dx%d", image_width, image_height);
                    display_debug_info(info_msg, false);
                    
                    // 重置缓冲区索引
                    // image_buffer_index = 0;
                    image_header_received = true;
                    // esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    // // 如果数据包中还有图像数据，则复制到缓冲区
                    // if (param->write.len > 8) {
                    //     uint32_t data_len = param->write.len - 8;
                    //     if (image_buffer_index + data_len <= IMAGE_BUFFER_SIZE) {
                    //         memcpy(image_buffer + image_buffer_index, param->write.value + 8, data_len);
                    //         image_buffer_index += data_len;
                            
                    //         // 显示接收进度
                    //         uint32_t expected_size = (image_width * image_height) / 2; // 假设4位灰度图，每个像素占4位
                    //         int percent = (image_buffer_index * 100) / expected_size;
                    //         sprintf(info_msg, "loading: %d%%", percent > 100 ? 100 : percent);
                    //         display_debug_info(info_msg, false);
                    //     } else {
                    //         display_debug_info("error: data large", false);
                    //     }
                    // }
                } else if (image_header_received) {
                    // 继续接收图像数据
                    image_width = *(uint8_t*)param->write.value;
                    sprintf(info_msg, "recive: %d", image_width);
                    display_debug_info(info_msg, false);
                    // esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    // if (image_buffer_index + param->write.len <= IMAGE_BUFFER_SIZE) {
                    //     memcpy(image_buffer + image_buffer_index, param->write.value, param->write.len);
                    //     image_buffer_index += param->write.len;
                        
                    //     // 显示接收进度
                    //     uint32_t expected_size = (image_width * image_height) / 2; // 假设4位灰度图，每个像素占4位
                    //     int percent = (image_buffer_index * 100) / expected_size;
                        
                    //     // 每接收一定量数据更新一次进度显示
                    //     if (param->write.len >= 1000 || percent >= 100) {
                    //         sprintf(info_msg, "loading: %d%%", percent > 100 ? 100 : percent);
                    //         display_debug_info(info_msg, false);
                    //     }
                        
                    //     // 检查是否接收完成
                    //     if (image_buffer_index >= expected_size) {
                    //         image_received_complete = true;
                    //         display_debug_info("done", false);
                            
                    //         // 在这里可以调用处理图像的函数
                    //         process_received_image();
                    //     }
                    // } else {
                    //     display_debug_info("error: max", false);
                    // }
                }
            }
        }
        break;
    default:
        break;
    }
}

// GATTS回调函数
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            image_profile_tab.gatts_if = gatts_if;
        } else {
            ESP_LOGI("GATTS", "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id, param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == image_profile_tab.gatts_if) {
            if (image_profile_tab.gatts_cb) {
                image_profile_tab.gatts_cb(event, gatts_if, param);
            }
        }
    } while (0);
}

// 初始化蓝牙
static void bluetooth_init(void) {
    esp_err_t ret;
    
    // 显示初始化开始信息
    // display_debug_info("bluetooth init", false);

    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // display_debug_info("NVS init", false);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ret = esp_bluedroid_init();

    ret = esp_bluedroid_enable();

    ret = esp_ble_gatts_register_callback(gatts_event_handler);

    ret = esp_ble_gap_register_callback(gap_event_handler);

    ret = esp_ble_gatts_app_register(0);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);

}

void idf_setup() {
    epd_init(&DEMO_BOARD, &ED060KD1, EPD_LUT_64K);

    epd_set_vcom(1560);

    hl = epd_hl_init(WAVEFORM);

    epd_set_rotation(EPD_ROT_LANDSCAPE);

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
    display_debug_info("", true);

    bluetooth_init();
    display_debug_info("bluetooth_init done", false);

    display_debug_info("", true);

    display_debug_info("hello world", false);

    display_debug_info("lismin", false);
}

void idf_loop() {
    
    if (image_received_complete) {
        process_received_image();
    }
    
    // 添加一些延迟，避免CPU占用过高
    delay(100);
}

#ifndef ARDUINO_ARCH_ESP32
void app_main() {
    idf_setup();
    while (1) {
        idf_loop();
    };
}
#endif