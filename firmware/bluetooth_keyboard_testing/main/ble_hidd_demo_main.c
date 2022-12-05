/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

//custom keyboard includes
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"


/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"


#define CONFIG_BLINK_LED_RMT_CHANNEL 0

//custom keyboard codes

#define BLINK_GPIO 11
//size of arrays
#define row_size (sizeof(row)/(sizeof(gpio_num_t)))
#define col_size (sizeof(col)/(sizeof(gpio_num_t)))

//used gpio-s
gpio_num_t row[3] = {15, 2, 1};
gpio_num_t col[4] = {7, 6, 5, 4};

// Indices of LEDs
uint8_t cl3_array[3] = {0, 7, 8};
uint8_t cl2_array[3] = {1, 6, 9};
uint8_t cl1_array[3] = {2, 5, 10};
uint8_t cl0_array[3] = {3, 4, 11};
uint8_t au8LedIndex[ row_size ][ col_size ] = { {3, 2, 1, 0}, 
                                                {4, 5, 6, 7}, 
                                                {11, 10, 9, 8}};

// Button state
bool bButtonStateCurr[ row_size ][ col_size ] = {{false, false, false, false}, 
                                                {false, false, false, false}, 
                                                {false, false, false, false}};;
bool bButtonStateOld[ row_size ][ col_size ] = {{false, false, false, false}, 
                                                {false, false, false, false}, 
                                                {false, false, false, false}};

uint8_t au8Key_Values[row_size][col_size] = {   {HID_KEY_F1,            HID_KEY_F2,         HID_KEY_F3,         HID_KEY_ESCAPE}, 
                                                {HID_KEY_A,             HID_KEY_UP_ARROW,   HID_KEY_DELETE_FWD, HID_KEY_LEFT_ALT }, 
                                                {HID_KEY_LEFT_ARROW,    HID_KEY_DOWN_ARROW, HID_KEY_RIGHT_ARROW,HID_KEY_LEFT_CTRL }};
uint8_t aBT_Key_State[12];
uint8_t u8NumKeysPressed = 0u;
uint8_t key_value;


static led_strip_t *pStrip_a;

static void blink_led(uint8_t led_index)
{
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        pStrip_a->set_pixel(pStrip_a, led_index, 16, 16, 16);
}


static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 12);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}


//Torok Samuel codes
static void conf_gpio_output(gpio_num_t *gpio_port_output, uint8_t index)
{    
    for(uint8_t i = 0; i < index; i++)
    {
        //Reset pin first
        gpio_reset_pin(gpio_port_output[i]);
        //Set GPIO to Output mode
        //gpio_intr_disable(gpio_port_output[i]);
        gpio_set_direction(gpio_port_output[i], GPIO_MODE_OUTPUT);
        //gpio_pulldown_dis(gpio_port_output[i]);
        //gpio_pullup_dis(gpio_port_output[i]);
    }        
} 

static void conf_gpio_input(gpio_num_t *gpio_port_input, uint8_t index)
{    
    for(uint8_t i = 0; i < index; i++)
    {
        //Reset pin first
        gpio_reset_pin(gpio_port_input[i]);
        //Set GPIO to Input mode
        gpio_set_direction(gpio_port_input[i], GPIO_MODE_INPUT);
        gpio_pulldown_dis(gpio_port_input[i]);
        gpio_pullup_en(gpio_port_input[i]);
    }        
} 

//end of custom keyboard codes


//bluetooth codes


static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1) {

        if (sec_conn) {
            
            
            for(int i = 0; i < row_size; i++)
            {
                    gpio_set_level(row[i], 0);
                    for( int j = 0; j < col_size; j++ )
                    {
                        bButtonStateCurr[ i ][ j ] = ( gpio_get_level( col[ j ] ) == 0 ) ? true : false;
                    }
                    gpio_set_level(row[i], 1);
            }


            for(int i = 0; i < row_size; i++)
            {
                    for( int j = 0; j < col_size; j++ )
                    {
                        //BT Key state
                        if( (true == bButtonStateCurr[i][j]) && (false == bButtonStateOld[i][j]) )
                        {
                            if((i == 1) && (j == 3))
                                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
                            else if((i == 2) && (j == 3))
                                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
                            else
                            {
                                key_value = au8Key_Values[i][j];
                                aBT_Key_State[ u8NumKeysPressed ] = key_value;
                                u8NumKeysPressed++;
                            }
                            
                            //ESP_LOGI(HID_DEMO_TAG, "Send a key");
                        }
                        else if( (false == bButtonStateCurr[i][j]) && (true == bButtonStateOld[i][j]) )     // false == bButtonStateCurr[i][j] && true == bButtonStateOld[i][j] 
                        {
                            if((i == 1) && (j == 3))
                                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                            else if((i == 2) && (j == 3))
                                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
                            else
                            {
                                key_value = au8Key_Values[i][j];
                            
                                for(int i = 0; i < u8NumKeysPressed; i++)
                                {                                
                                    u8NumKeysPressed--;
                                    if(aBT_Key_State[ u8NumKeysPressed ] == key_value)
                                    {
                                        for(int j = i; j < u8NumKeysPressed; j++)
                                        {
                                            aBT_Key_State[j] = aBT_Key_State[j+1];
                                            //ESP_LOGI(HID_DEMO_TAG, "Dont send that key");
                                        }                                        
                                    }                                
                                }
                            }
                            
                        }
                        
                        //LED state
                        if( true == bButtonStateCurr[i][j] )
                            pStrip_a->set_pixel( pStrip_a, au8LedIndex[i][j], 16, 16, 16);
                        else
                            pStrip_a->set_pixel( pStrip_a, au8LedIndex[i][j], 0, 0, 0);
                    }
            }
            pStrip_a->refresh(pStrip_a, 100);
            //TODO: BT küldés
            if( 0 == u8NumKeysPressed )
            {
                uint8_t key_vaule2 = {HID_KEY_RESERVED};
                esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule2, 1);
            }
            else
            {
                esp_hidd_send_keyboard_value(hid_conn_id, 0, &aBT_Key_State, u8NumKeysPressed);
            }

            //testing states
            //ESP_LOGI(HID_DEMO_TAG, "%s %s", bButtonStateCurr[1][0] ? "currtrue" : "currfalse",bButtonStateOld[1][0] ? "oldtrue" : "oldfalse" );

            for(int i = 0; i < row_size; i++)
            {
                    for( int j = 0; j < col_size; j++ )
                    {
                        bButtonStateOld[ i ][ j ] = bButtonStateCurr[ i ][ j ];
                    }
            }
            
       

            vTaskDelay(100 / portTICK_PERIOD_MS);

            //uint8_t key_vaule = {HID_KEY_A};
            //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);


        }
    }
}

void app_main(void)
{

    //keyboard codes



    /* Configure the peripheral according to the LED type */
    configure_led();
  
    //Configure the keys GPIO
    conf_gpio_output(row, 3);
    conf_gpio_input(col, 4);


    //bluetooth codes

    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}


