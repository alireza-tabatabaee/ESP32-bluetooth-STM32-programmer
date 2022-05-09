#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
//#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

// static struct timeval time_new, time_old;
// static long data_num = 0;

// state machine. 0 = waiting for connection 
    // 1 - waiting to recieve length
    // 2 - recieving data
uint8_t progState = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

uint32_t gHandle;
uint16_t dataLen = 0;
uint8_t bufPtr = 0;
char buf[5][256];
TaskHandle_t parseTaskHandle;
QueueHandle_t packetQueue;

void eraseFlash()
{
    //uint8_t hddt[2] = {0x43, 0xBC};
    uint8_t hddt[2] = {0x44, 0xBB};
    uart_write_bytes(UART_NUM_1, &hddt, 2);
    uint8_t ackstat = 0;
    uart_read_bytes(UART_NUM_1, &ackstat, 1, 2000);
    ESP_LOGW("EXERACK", "%d", ackstat);
    if(ackstat==0x79)
    {
        uint8_t masserase [3] = {0xFF, 0xFF, 0x00};
        uart_write_bytes(UART_NUM_1, &masserase, 3);
    }
    uart_read_bytes(UART_NUM_1, &ackstat, 1, 2000);
    ESP_LOGW("ERASEMASS", "%d", ackstat);
}

uint32_t flashadd = 0x08000000;
void writeToMem(uint8_t len, uint8_t bufId)
{
    uint8_t hddt[2] = {0x31, 0xCE};
    uart_write_bytes(UART_NUM_1, &hddt, 2);
    uint8_t ackstat = 0;
    uart_read_bytes(UART_NUM_1, &ackstat, 1, 2000);
    ESP_LOGW("WR1", "%d", ackstat);
    if(ackstat==0x79)
    {
        ackstat = 0;
        uint8_t flashaddbytes[5]; 
        flashaddbytes[0] = (flashadd>>24);
        flashaddbytes[1] = (flashadd&0x00FF0000) >> 16;
        flashaddbytes[2] = (flashadd&0x0000FF00) >> 8;
        flashaddbytes[3] = (flashadd&0x000000FF);
        flashaddbytes[4] = (flashaddbytes[0]^flashaddbytes[1]^flashaddbytes[2]^flashaddbytes[3]);
        uart_write_bytes(UART_NUM_1, flashaddbytes, 5);
        uart_read_bytes(UART_NUM_1, &ackstat, 1, 2000);
        ESP_LOGW("WR2", "%d", ackstat);
        if(ackstat==0x79)
        {
            ackstat = 0;
            uint8_t len1 = len;// - 1;
            uint8_t checksumf = len1;
            for(int i=0; i<len+1; i++)
                checksumf ^= buf[bufId][i];
            uart_write_bytes(UART_NUM_1, &len1, 1);
            uart_write_bytes(UART_NUM_1, buf[bufId], len+1);
            uart_write_bytes(UART_NUM_1, &checksumf, 1);
            uart_read_bytes (UART_NUM_1, &ackstat, 1, 2000);
            ESP_LOGW("WR3", "%d", ackstat);
            if(ackstat==0x79){
                flashadd += ((uint32_t)len+1);
                esp_spp_write(gHandle, 7, (uint8_t*)"DATSENT");
                ESP_LOGW("FL", "FL: %#X", flashadd);
            }
            else
                esp_spp_write(gHandle, 5, (uint8_t*)"ERRW3");
        }
        else
            esp_spp_write(gHandle, 5, (uint8_t*)"ERRW2");
    }
    else
        esp_spp_write(gHandle, 5,(uint8_t*) "ERRW1");
}

void parseTask(void* inp)
{
    ESP_LOGW("Q", "startpars success");
    while(1)
    {
        uint32_t notificationvalue = ulTaskNotifyTake( pdTRUE, 100 );
        if(notificationvalue==2 && uxQueueMessagesWaiting(packetQueue)==0)
        {
            ESP_LOGW("Q", "parse task bye!!!");
            vTaskDelete(NULL);
        } 
        else {
            uint8_t len, parseid;
            xQueueReceive(packetQueue, &len, 3000);
            xQueueReceive(packetQueue, &parseid, 3000);
            writeToMem(len, parseid);
            ESP_LOGW("QA", "%d bytes", len);
            ESP_LOGW("QA", "%s", buf[parseid] );
        }
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name("ESP32 STM programmer");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    // data input event
    case ESP_SPP_DATA_IND_EVT:

        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d",
                 param->data_ind.len, param->data_ind.handle);
        if(strcmp((char*)param->data_ind.data, "RES")==0)
            progState = 0;
        else if(progState == 0)
        {
            if(strcmp((char*)param->data_ind.data, "`INIT")==0)
            {
                // reset the STM32 MCU by sending a negative pulse on the NRST pin
                gpio_set_level(GPIO_NUM_5, 0);
                vTaskDelay(25);
                gpio_set_level(GPIO_NUM_5, 1);
                vTaskDelay(10);
                // send 0x7F to begin communication with the System Memory bootloader
                uint8_t n7f = 0x7F;
                uart_write_bytes(UART_NUM_1, &n7f, 1);             
                uart_read_bytes(UART_NUM_1, &n7f, 1, 2000);
                if(n7f == 0x79){
                    // create the task that sends the flashing data over UART
                    BaseType_t ret = xTaskCreate(parseTask, "Parsing info and sending w UART", 2048, NULL, 0, &parseTaskHandle);
                    if(ret!=pdPASS)
                        ESP_LOGW("Q", "TaskCreFail");
                    else
                        ESP_LOGW("Q", "Task creat");
                    esp_spp_write(gHandle, 4, (uint8_t*)"ACK0");
                    progState = 1;
                    eraseFlash();
                }
                else
                   esp_spp_write(gHandle, 1, &n7f); 
            }
            else
                esp_spp_write(gHandle, 5, (uint8_t*)"ERRHE");
        }
        else if(progState==1)
        {
            if (strcmp((char*)param->data_ind.data, "`FINI"))
            {
                dataLen = atoi((char*)param->data_ind.data);
                if(dataLen>0 && dataLen<=256)
                {
                    xQueueSend(packetQueue, &dataLen, 3000);
                    esp_spp_write(gHandle, 4, (uint8_t*)"ACK1");
                    progState = 2;
                }
                else
                esp_spp_write(gHandle, 5, (uint8_t*)"ERRLN"); 
            }
            else { // End of transmission
                xTaskNotify(parseTaskHandle, 2, eSetValueWithOverwrite);
                progState = 0;
            }
        }
        else if(progState == 2)
        {
            if (bufPtr == 5) bufPtr = 0;
            strcpy(buf[bufPtr] , (char*)param->data_ind.data);
            xQueueSend(packetQueue, &bufPtr, 3000);
            bufPtr++;
            dataLen = 0;
            progState = 1;                
        }    

        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        gHandle = param->srv_open.handle;
        //gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void app_main(void)
{
    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    //esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    //esp_bt_pin_code_t pin_code;
    //esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    gpio_config_t gconf = {.pin_bit_mask=1<<5, .mode=GPIO_MODE_OUTPUT, .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,};
    gpio_config(&gconf);
    gpio_set_level(GPIO_NUM_5, 1);
    uart_config_t uconf = {.baud_rate = 38400, .data_bits= UART_DATA_8_BITS, .parity = UART_PARITY_EVEN, .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .rx_flow_ctrl_thresh = 1,};
    uart_param_config(UART_NUM_1,&uconf);
    uart_set_pin(UART_NUM_1, 17, 16, 18, 19); //TX, RX, CTS, RTS
    uart_driver_install(UART_NUM_1,260,260,2,NULL,0);

    packetQueue = xQueueCreate(10, sizeof(uint8_t));
}