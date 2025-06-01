/* Simple firmware for a ESP32 displaying a static image on an EPaper Screen.
 *
 * Write an image into a header file using a 3...2...1...0 format per pixel,
 * for 4 bits color (16 colors - well, greys.) MSB first.  At 80 MHz, screen
 * clears execute in 1.075 seconds and images are drawn in 1.531 seconds.
 */

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
#include <nvs.h>
// 添加蓝牙相关头文件
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gatts_api.h>
#include <esp_system.h>

#include <epdiy.h>

#include "sdkconfig.h"

#include "firasans_12.h"
#include "firasans_20.h"
#include "img_board.h"

#define WAVEFORM EPD_BUILTIN_WAVEFORM

#define epd_poweron() digitalWrite(46,1)
#define epd_poweroff() digitalWrite(46,0)

// 蓝牙相关定义
#define GATTS_SERVICE_UUID_IMAGE   0x00FF
#define GATTS_CHAR_UUID_IMAGE_DATA 0xFF01
#define GATTS_NUM_HANDLE_IMAGE     4

#define DEVICE_NAME "ESP32-EPaper"
#define MANUFACTURER_DATA_LEN  4

// 图像缓冲区定义
#define MAX_IMAGE_SIZE (700 * 396 / 2) // 根据img_board.h中的图像大小定义
static uint8_t received_image_data[MAX_IMAGE_SIZE];
static uint32_t received_image_width = 700;
static uint32_t received_image_height = 396;
static uint32_t received_data_length = 0;
static bool image_received_complete = false;

// 蓝牙服务和特征句柄
static uint16_t image_handle_table[GATTS_NUM_HANDLE_IMAGE];
static esp_gatt_if_t image_gatts_if;

// choose the default demo board depending on the architecture
#ifdef CONFIG_IDF_TARGET_ESP32
#define DEMO_BOARD epd_board_v6
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define DEMO_BOARD epd_board_v7
#endif

EpdiyHighlevelState hl;


const EpdDisplay_t ES080FC = {
    .width = 1800,
    .height = 600,
    .bus_width = 16,
    .bus_speed = 17,
    .default_waveform = &epdiy_ED097TC2,
    .display_type = DISPLAY_TYPE_GENERIC,
};
const EpdDisplay_t ES108FC = {
    .width = 1920,
    .height = 1080,
    .bus_width = 16,
    .bus_speed = 17,
    .default_waveform = &epdiy_ED047TC1,
    .display_type = DISPLAY_TYPE_GENERIC,
};
const EpdDisplay_t ED060KD1 = {
    .width = 1448,
    .height = 1072,
    .bus_width = 8,
    .bus_speed = 20,
    .default_waveform = &epdiy_ED060SCT,
    .display_type = DISPLAY_TYPE_GENERIC,
};

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


// Add this function declaration before it's first used (before process_received_image function)
static inline void checkError(enum EpdDrawError err) {
    if (err != EPD_DRAW_SUCCESS) {
        ESP_LOGE("demo", "draw error: %X", err);
    }
}

// 蓝牙广播参数
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
// 显示调试信息的通用函数
static void display_debug_info(const char* title, const char* message, bool clear_screen) {
    uint8_t* fb = epd_hl_get_framebuffer(&hl);
    int temperature = epd_ambient_temperature();
    
    if (clear_screen) {
        epd_clear();
    }
    
    // 设置文本位置和属性
    int cursor_x = 10;
    int cursor_y = 30;
    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    
    // 显示标题
    if (title != NULL) {
        epd_write_string(&FiraSans_20, title, &cursor_x, &cursor_y, fb, &font_props);
        cursor_y += 30;
    }
    
    // 显示消息
    if (message != NULL) {
        cursor_x = 10;
        epd_write_string(&FiraSans_12, message, &cursor_x, &cursor_y, fb, &font_props);
    }
    
    // 显示时间戳
    cursor_x = 10;
    cursor_y = epd_rotated_display_height() - 20;
    char timestamp[32];
    sprintf(timestamp, "时间: %lld 秒", esp_timer_get_time() / 1000000);
    epd_write_string(&FiraSans_12, timestamp, &cursor_x, &cursor_y, fb, &font_props);
    
    // 更新屏幕
    checkError(epd_hl_update_screen(&hl, MODE_GC16, temperature));

    
    // 同时输出到控制台
    ESP_LOGI("DEBUG", "%s: %s", title ? title : "信息", message ? message : "");
}

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
        display_debug_info("蓝牙广播", "广播数据设置完成，开始广播", true);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            char error_msg[64];
            sprintf(error_msg, "广播启动失败: %d", param->adv_start_cmpl.status);
            display_debug_info("错误", error_msg, false);
        } else {
            display_debug_info("蓝牙广播", "广播成功启动，设备名称: " DEVICE_NAME "", true);
        }
        break;
    default:
        break;
    }
}

// 处理接收到的图像数据
static void process_received_image() {
    if (image_received_complete) {
        // 获取帧缓冲区
        uint8_t* fb = epd_hl_get_framebuffer(&hl);
        
        // 获取温度
        epd_poweron();
        epd_clear();
        int temperature = epd_ambient_temperature();
        epd_poweroff();
        
        // 显示处理开始信息
        char info_msg[128];
        sprintf(info_msg, "图像尺寸: %dx%d, 数据大小: %d字节, 温度: %d°C", 
                received_image_width, received_image_height, received_data_length, temperature);
        display_debug_info("图像处理", info_msg, true);
        
        // 检查图像数据有效性
        if (received_data_length == 0 || 
            received_data_length != (received_image_width * received_image_height) / 2) {
            
            char error_msg[128];
            sprintf(error_msg, "图像数据无效! 预期: %d字节, 实际: %d字节", 
                    (received_image_width * received_image_height) / 2, 
                    received_data_length);
            display_debug_info("错误", error_msg, true);
            
            // 重置接收计数器
            received_data_length = 0;
            image_received_complete = false;
            return;
        }
        
        // 检查图像尺寸是否合适
        if (received_image_width > epd_rotated_display_width() || 
            received_image_height > epd_rotated_display_height()) {
            
            char error_msg[128];
            sprintf(error_msg, "图像尺寸过大! 图像: %dx%d, 屏幕: %dx%d", 
                    received_image_width, received_image_height,
                    epd_rotated_display_width(), epd_rotated_display_height());
            display_debug_info("错误", error_msg, true);
            
            // 重置接收计数器
            received_data_length = 0;
            image_received_complete = false;
            return;
        }
        
        // 显示处理中信息
        display_debug_info("图像处理", "正在准备显示区域...", true);
        
        // 准备显示图像
        epd_hl_set_all_white(&hl);
        
        // 计算图像位置（居中显示）
        EpdRect image_area = {
            .x = epd_rotated_display_width() / 2 - received_image_width / 2,
            .y = epd_rotated_display_height() / 2 - received_image_height / 2,
            .width = received_image_width,
            .height = received_image_height,
        };
        
        // 显示图像区域信息
        sprintf(info_msg, "图像区域: x=%d, y=%d, 宽=%d, 高=%d", 
                image_area.x, image_area.y, image_area.width, image_area.height);
        display_debug_info("图像处理", info_msg, true);
        
        // 显示复制数据提示
        display_debug_info("图像处理", "正在复制图像数据到帧缓冲区...", true);
        
        // 复制图像数据到帧缓冲区
        epd_hl_set_all_white(&hl);
        
        // 尝试复制图像数据
        bool copy_success = true;
        
        // 使用try-catch模式捕获可能的错误
        if (received_image_width <= epd_rotated_display_width() && 
            received_image_height <= epd_rotated_display_height() &&
            received_data_length > 0) {
            
            // 使用接收到的图像数据
            epd_copy_to_framebuffer(image_area, received_image_data, fb);
        } else {
            copy_success = false;
        }
        
        // 检查复制结果
        if (!copy_success) {
            display_debug_info("错误", "图像数据复制失败!", true);
            
            // 重置接收计数器
            received_data_length = 0;
            image_received_complete = false;
            return;
        }
        
        // 显示更新屏幕提示
        display_debug_info("图像处理", "正在更新屏幕显示...", true);
        
        // 最终更新屏幕，显示图像
        epd_poweron();
        enum EpdDrawError update_result = epd_hl_update_screen(&hl, MODE_GC16, temperature);
        
        // 检查更新结果
        if (update_result != EPD_DRAW_SUCCESS) {
            char error_msg[64];
            sprintf(error_msg, "屏幕更新失败! 错误代码: %X", update_result);
            display_debug_info("错误", error_msg, true);
        } else {
            display_debug_info("图像处理", "图像显示成功!", true);
        }
        
        epd_poweroff();
        
        // 更新完成后，在控制台输出日志
        ESP_LOGI("BLE", "图像显示完成: %dx%d (%d字节)", 
                received_image_width, received_image_height, received_data_length);
        
        // 重置接收计数器
        received_data_length = 0;
        image_received_complete = false;
    }
}

// GATT服务回调函数实现
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        {
            char info_msg[64];
            sprintf(info_msg, "应用注册状态: %d, 应用ID: %d", param->reg.status, param->reg.app_id);
            display_debug_info("GATTS注册", info_msg, true);
            
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
            char info_msg[64];
            sprintf(info_msg, "服务创建状态: %d, 服务句柄: %d", param->create.status, param->create.service_handle);
            display_debug_info("GATTS服务", info_msg, true);
            
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
            char info_msg[128];
            sprintf(info_msg, "特征添加状态: %d, 属性句柄: %d, 服务句柄: %d", 
                    param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            display_debug_info("GATTS特征", info_msg, true);
            
            image_profile_tab.char_handle = param->add_char.attr_handle;
            
            // 启动服务
            esp_ble_gatts_start_service(image_profile_tab.service_handle);
        }
        break;
    case ESP_GATTS_START_EVT:
        {
            char info_msg[64];
            sprintf(info_msg, "服务启动状态: %d, 服务句柄: %d", 
                    param->start.status, param->start.service_handle);
            display_debug_info("GATTS服务", info_msg, true);
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        {
            char info_msg[128];
            sprintf(info_msg, "连接ID: %d, 设备地址: %02x:%02x:%02x:%02x:%02x:%02x",
                    param->connect.conn_id,
                    param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                    param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            display_debug_info("蓝牙连接", info_msg, true);
            
            image_profile_tab.conn_id = param->connect.conn_id;
        }
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        {
            char reason_str[32] = "未知原因";
            switch(param->disconnect.reason) {
                case 0x13: strcpy(reason_str, "用户终止连接"); break;
                case 0x16: strcpy(reason_str, "连接超时"); break;
                case 0x22: strcpy(reason_str, "对方设备终止连接"); break;
                case 0x08: strcpy(reason_str, "监督超时"); break;
                default: sprintf(reason_str, "代码: 0x%x", param->disconnect.reason); break;
            }
            
            char info_msg[64];
            sprintf(info_msg, "断开连接，原因: %s", reason_str);
            display_debug_info("蓝牙断开", info_msg, true);
            
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == image_profile_tab.char_handle) {
            // 处理图像数据写入
            if (param->write.len > 0) {
                // 前8个字节包含图像宽度和高度信息
                if (received_data_length == 0 && param->write.len >= 8) {
                    received_image_width = *(uint32_t*)param->write.value;
                    received_image_height = *(uint32_t*)(param->write.value + 4);
                    
                    // 显示图像信息
                    char info_msg[128];
                    sprintf(info_msg, "接收图像数据开始，尺寸: %dx%d", 
                            received_image_width, received_image_height);
                    display_debug_info("数据接收", info_msg, true);
                    
                    // 复制剩余的图像数据
                    if (param->write.len > 8) {
                        memcpy(received_image_data, param->write.value + 8, param->write.len - 8);
                        received_data_length = param->write.len - 8;
                        
                        // 更新进度
                        char progress_msg[64];
                        sprintf(progress_msg, "已接收: %d/%d 字节 (%.1f%%)", 
                                received_data_length, 
                                (received_image_width * received_image_height) / 2,
                                (float)received_data_length * 100.0 / ((received_image_width * received_image_height) / 2));
                        display_debug_info("接收进度", progress_msg, false);
                    }
                } else {
                    // 确保不会溢出缓冲区
                    if (received_data_length + param->write.len <= MAX_IMAGE_SIZE) {
                        memcpy(received_image_data + received_data_length, param->write.value, param->write.len);
                        received_data_length += param->write.len;
                        
                        // 每接收一定量的数据更新一次进度
                        if (param->write.len >= 1000 || 
                            received_data_length >= (received_image_width * received_image_height) / 2) {
                            char progress_msg[64];
                            sprintf(progress_msg, "已接收: %d/%d 字节 (%.1f%%)", 
                                    received_data_length, 
                                    (received_image_width * received_image_height) / 2,
                                    (float)received_data_length * 100.0 / ((received_image_width * received_image_height) / 2));
                            display_debug_info("接收进度", progress_msg, false);
                        }
                    } else {
                        char error_msg[128];
                        sprintf(error_msg, "缓冲区溢出! 当前: %d, 新增: %d, 最大: %d", 
                                received_data_length, param->write.len, MAX_IMAGE_SIZE);
                        display_debug_info("错误", error_msg, true);
                    }
                }
                
                // 检查是否接收完成
                if (received_data_length >= (received_image_width * received_image_height) / 2) {
                    display_debug_info("数据接收", "图像数据接收完成，准备处理图像", true);
                    image_received_complete = true;
                    process_received_image();
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
    display_debug_info("蓝牙初始化", "正在初始化蓝牙...", true);

    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        display_debug_info("NVS初始化", "需要擦除NVS并重新初始化", false);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        char error_msg[64];
        sprintf(error_msg, "NVS初始化失败: %s", esp_err_to_name(ret));
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("NVS初始化", "NVS初始化成功", false);
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "初始化控制器失败: %s", esp_err_to_name(ret));
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("BT控制器", "控制器初始化成功", false);
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "启用控制器失败: %s", esp_err_to_name(ret));
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("BT控制器", "控制器启用成功", false);
    }

    ret = esp_bluedroid_init();
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "初始化蓝牙协议栈失败: %s", esp_err_to_name(ret));
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("蓝牙协议栈", "协议栈初始化成功", false);
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "启用蓝牙协议栈失败: %s", esp_err_to_name(ret));
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("蓝牙协议栈", "协议栈启用成功", false);
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "GATTS回调注册失败: %x", ret);
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("GATTS", "GATTS回调注册成功", false);
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "GAP回调注册失败: %x", ret);
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("GAP", "GAP回调注册成功", false);
    }

    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        char error_msg[64];
        sprintf(error_msg, "GATTS应用注册失败: %x", ret);
        display_debug_info("错误", error_msg, false);
        return;
    } else {
        display_debug_info("GATTS", "GATTS应用注册成功", false);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        char error_msg[64];
        sprintf(error_msg, "设置本地MTU失败: %x", local_mtu_ret);
        display_debug_info("错误", error_msg, false);
    } else {
        display_debug_info("GATT", "本地MTU设置为500", false);
    }
    
    // 显示初始化完成信息
    display_debug_info("蓝牙初始化", "蓝牙初始化完成，等待连接...", true);
}

void idf_setup() {
    epd_init(&DEMO_BOARD, &ED060KD1, EPD_LUT_64K);

    /*     
    Documents\Arduino\libraries\epdiy2\src\display.c  中修改
    */

    
    // Set VCOM for boards that allow to set this in software (in mV).
    // This will print an error if unsupported. In this case,
    // set VCOM using the hardware potentiometer and delete this line.
    epd_set_vcom(1560);

    hl = epd_hl_init(WAVEFORM);

    // Default orientation is EPD_ROT_LANDSCAPE
    epd_set_rotation(EPD_ROT_LANDSCAPE);

    printf(
        "Dimensions after rotation, width: %d height: %d\n\n", epd_rotated_display_width(),
        epd_rotated_display_height()
    );

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
    
    // 初始化蓝牙
    bluetooth_init();
}

#ifndef ARDUINO_ARCH_ESP32
void delay(uint32_t millis) {
    vTaskDelay(millis / portTICK_PERIOD_MS);
}
#endif


void idf_loop() {
    static uint32_t last_status_update = 0;
    uint32_t current_time = esp_timer_get_time() / 1000000; // 转换为秒
    
    // 每30秒更新一次状态信息（如果没有正在处理的图像）
    if (!image_received_complete && (current_time - last_status_update >= 30 || last_status_update == 0)) {
        char status_msg[128];
        sprintf(status_msg, "运行时间: %u秒\n蓝牙状态: %s\n连接ID: %d", 
                current_time,
                image_profile_tab.conn_id ? "已连接" : "等待连接",
                image_profile_tab.conn_id);
        
        display_debug_info("系统状态", status_msg, true);
        
        last_status_update = current_time;
    }
    
    // 如果有新图像接收完成，处理图像
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
