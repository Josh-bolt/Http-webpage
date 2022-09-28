/* TWAI Network Master Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sys/param.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <esp_http_server.h>
/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "REVOS"
#define EXAMPLE_ESP_WIFI_PASS      "revos@1234"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;
/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          250
#define NO_OF_DATA_MSGS         1
#define NO_OF_ITERS             1
#define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define TX_GPIO_NUM             GPIO_NUM_5
#define RX_GPIO_NUM             GPIO_NUM_32
#define EXAMPLE_TAG             "TWAI Master"

#define TX_PIN_CHARGER GPIO_NUM_4
#define RX_PIN_CHARGER GPIO_NUM_33


#define CHARGER_WAKEUP_ID       0x500
#define CHARGER_ACC_ID          0x508
#define SET_CHARGER_VAL_ID      0x501
#define GET_CHARGER_VAL_ID      0x509

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

// static const twai_timing_config_t tim_config = TWAI_TIMING_CONFIG_500KBITS();
// static const twai_filter_config_t fil_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// static const twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN_CHARGER,RX_PIN_CHARGER, TWAI_MODE_NORMAL);
static twai_message_t tx_message = {.identifier = CHARGER_WAKEUP_ID, .data_length_code = 1,
                                            .data = {0, 0, 0, 0, 0, 0, 0, 0}};
static twai_message_t tx_data_message = {.identifier = SET_CHARGER_VAL_ID, .data_length_code = 8,
                                            .data = {0x02, 0x55, 0x01, 0x90, 1, 0, 0, 0}};
//static twai_message_t can_tx_message = {.identifier = ID_SPEEDOMETER_MOTOR_CTRL1_DATA, .data_length_code = 8, .extd = 1, .ss = 1, .data = {0, 0, 0, 0, 0, 0, 0, 0}};

static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t rx_task_sem;
bool acc_pass = false;
static uint8_t count = 0;
struct chargerdata {
    uint8_t BATTERY_VOL_MSB;
    uint8_t BATTERY_VOL_LSB;
    uint8_t BATTERY_CUR_MSB;
    uint8_t BATTERY_CUR_LSB;
    uint8_t CHARGER_STATUS;
    uint8_t CHARGER_FAULTS;
};
struct chargerdata rx_chargerdata;
 uint16_t BATTERY_VOLTAGE;
uint16_t BATTERY_CURRENT;

extern Battery_vol_msb;
extern const uint8_t jquery_3_3_1_min_js_start[]	asm("_binary_jquery_3_3_1_min_js_start");
extern const uint8_t jquery_3_3_1_min_js_end[]		asm("_binary_jquery_3_3_1_min_js_end");
extern const uint8_t index_html_start[]				asm("_binary_index_html_start");
extern const uint8_t index_html_end[]				asm("_binary_index_html_end");
extern const uint8_t main_js_start[]				asm("_binary_main_js_start");
extern const uint8_t main_js_end[]					asm("_binary_main_js_end");
/* --------------------------- Tasks and Functions -------------------------- */
void CAN_RECEIVE_DATA(uint8_t *data, uint8_t len){
    rx_chargerdata.BATTERY_VOL_MSB = data[0];
    rx_chargerdata.BATTERY_VOL_LSB = data[1];
    //printf("%x, %x", (data[0] << 0x08) ,rx_chargerdata.BATTERY_VOL_LSB);
    BATTERY_VOLTAGE =  (data[1] | (data[0] << 0x08));
   // BATTERY_VOLTAGE = BATTERY_VOLTAGE/100;
    if(BATTERY_VOLTAGE != 0){
         printf("Battery voltage is %d\n", BATTERY_VOLTAGE);
    }
    rx_chargerdata.BATTERY_CUR_MSB =  data[2];
    rx_chargerdata.BATTERY_CUR_LSB = data[3];
    BATTERY_CURRENT =  (data[3] | (data[2] << 0x08));
    if(BATTERY_CURRENT != 0){
        printf("charger current is %d", BATTERY_CURRENT);
    }
    rx_chargerdata.CHARGER_STATUS = data[4];
    rx_chargerdata.CHARGER_FAULTS = data[5];
   // ESP_ERROR_CHECK(twai_transmit(&data, portMAX_DELAY));

}
void CAN_Transmit_Process_Task(void *arg)
{ 
    while(1){
        if(acc_pass){
          //  xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
          if(count < 5)  {
            tx_message.identifier = CHARGER_WAKEUP_ID;
            tx_message.data_length_code = 1;
            tx_message.data[0] = 0x55;
            ESP_ERROR_CHECK(twai_transmit(&tx_message, portMAX_DELAY));
            ESP_LOGI(EXAMPLE_TAG, "tx done");
            vTaskDelay(100);
          }
           // count++;
      //  vTaskDelay(10);
      //  xSemaphoreGive(ctrl_task_sem);
      //  xSemaphoreGive(rx_task_sem);
       // if(acc_pass){
            // tx_message.identifier = SET_CHARGER_VAL_ID;
            // tx_message.data_length_code = 8;
            // tx_message.data = {0x02, 0x44, 0x01, 0x90, 0, 0, 0, 0};
            ESP_ERROR_CHECK(twai_transmit(&tx_data_message, portMAX_DELAY));
            vTaskDelay(100);
        }

    }

}

void CAN_Receive_Process_Task(void *arg){
    twai_message_t rx_msg;
    while(1){
        ESP_LOGI(EXAMPLE_TAG, "rx while loop");
       // xSemaphoreTake(rx_task_sem, portMAX_DELAY);
        twai_receive(&rx_msg, portMAX_DELAY);
        switch(rx_msg.identifier){
            case CHARGER_ACC_ID:
                if(rx_msg.data[0] == 0xAA){
                    acc_pass = true;
                }
                break;
            case GET_CHARGER_VAL_ID:
                CAN_RECEIVE_DATA(&rx_msg.data[0], rx_msg.data_length_code);
                acc_pass = true;
                count++;  
                break;
            default:
                count = 0;
                acc_pass = false;
                ESP_LOGI(EXAMPLE_TAG, "in default");

                //xSemaphoreGive(ctrl_task_sem);
                break;
        }

    }

}

/**
 * app.css get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_index_html_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "index.html requested");

	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);

	return ESP_OK;
}

static esp_err_t http_server_jquery_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Jquery requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)jquery_3_3_1_min_js_start, jquery_3_3_1_min_js_end - jquery_3_3_1_min_js_start);

	return ESP_OK;
}
/**
 * app.js get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_app_js_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "main.js requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)main_js_start, main_js_end - main_js_start);

	return ESP_OK;
}

static esp_err_t http_server_candata_js_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "candata.js requested");

    char candata[100];

    sprintf(candata, "{\"battery_voltage\":\"%d\",\"battery_current\":\"%d\",\"charger_state\":\"%d\",\"bms_fault\":\"%d\"}",
    BATTERY_VOLTAGE,BATTERY_CURRENT,rx_chargerdata.CHARGER_STATUS,rx_chargerdata.CHARGER_FAULTS);
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, candata, strlen(candata));

    return ESP_OK;

}

static esp_err_t http_server_CAN_update_handler(httpd_req_t *req){
    ESP_LOGI(TAG, "input_data requested");

    char candata[100];
    httpd_resp_set_type(req, "application/javascript");
    int content_length = req->content_len;
	int content_received = 0;
	int recv_len;
    recv_len = httpd_req_recv(req, candata, MIN(content_length, sizeof(candata)));
    printf("%d", Battery_vol_msb.value);

    return ESP_OK;
}

// /* Our URI handler function to be called during GET /uri request */
// esp_err_t get_title_handler(httpd_req_t *req)
// {
//     /* Send a simple response */
//     const char resp[] = "<h1>Bolt</h1>""<h2>Charger CAN data</h2>""<p>The following displays the charger CAN ids data val:</p>"
//     "<script>document.write('BATTERY_VOLTAGE')</script>";
//    // const char resp[] = "uri get respondse";
//     uint8_t resp_len = strlen(resp);
//     httpd_resp_set_type(req, "text/html");
//     httpd_resp_send(req, resp, resp_len);
//     //  const char resp1[] = "<p>The following displays the charger CAN ids data val:</p>";
//     // httpd_resp_send(req, resp1, HTTPD_RESP_USE_STRLEN);
//     return ESP_OK;
// }

// /* URI handler structure for GET /uri */
// httpd_uri_t uri_get = {
//     .uri      = "/",
//     .method   = HTTP_GET,
//     .handler  = get_title_handler,
//     .user_ctx = NULL
// };

// esp_err_t get_handler(httpd_req_t *req)
// {
//     /* Send a simple response */
//    // const char resp[] = "<h1>Bolt</h1>""<h2>Charger CAN data</h2>""<p>The following displays the charger CAN ids data val:</p>";
//     const char resp[] = "<p> Voltage sensed: </p>";
//     uint8_t resp_len = strlen(resp);
//     httpd_resp_set_type(req, "text/html");
//     httpd_resp_send(req, resp, resp_len);
//     //  const char resp1[] = "<p>The following displays the charger CAN ids data val:</p>";
//     // httpd_resp_send(req, resp1, HTTPD_RESP_USE_STRLEN);
//     return ESP_OK;
// }

//register index.html handler
httpd_uri_t indexhtml = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_server_index_html_handler,
        .user_ctx = NULL
};
// register query handler
httpd_uri_t jquery_js = {
        .uri = "/jquery-3.3.1.min.js",
        .method = HTTP_GET,
        .handler = http_server_jquery_handler,
        .user_ctx = NULL
};
// register app.js handler
httpd_uri_t app_js = {
        .uri = "/main.js",
        .method = HTTP_GET,
        .handler = http_server_app_js_handler,
        .user_ctx = NULL
};

// register candata.json handler
httpd_uri_t can_data_json = {
    .uri = "/candata.json",
    .method = HTTP_GET,
    .handler = http_server_candata_js_handler,
    .user_ctx = NULL
};

// register OTAstatus handler
httpd_uri_t Update_status = {
        .uri = "/updateCAN",
        .method = HTTP_POST,
        .handler = http_server_CAN_update_handler,
        .user_ctx = NULL
};
// /* URI handler structure for GET /uri */
// httpd_uri_t t_get = {
//     .uri      = "/",
//     .method   = HTTP_GET,
//     .handler  = get_handler,
//     .user_ctx = NULL
// };
/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req)
{
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[100];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    printf("post content: %s\n", content);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    /* Send a simple response */
    // const char resp[] = "rece uari";
    //httpd_resp_set_type(req, "text/html");
    // httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}



/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {
    .uri      = "/",
    .method   = HTTP_POST,
    .handler  = post_handler,
    .user_ctx = NULL
};

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        // /* Register URI handlers */
        // httpd_register_uri_handler(server, &uri_get);
        // httpd_register_uri_handler(server, &uri_post);
        httpd_register_uri_handler(server, &indexhtml);//
        httpd_register_uri_handler(server, &jquery_js);
        httpd_register_uri_handler(server, &app_js);
        httpd_register_uri_handler(server, &can_data_json);//OTA_status
      //  httpd_register_uri_handler(server, &Update_status);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
    if (server) {
        /* Stop the httpd server */
        httpd_stop(server);
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void app_main(void)
{
    ctrl_task_sem = xSemaphoreCreateBinary();
    rx_task_sem = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
 
    ESP_LOGI(EXAMPLE_TAG, "CAN Driver Installed and started");
    //  ESP_ERROR_CHECK(twai_driver_install(&gen_config, &tim_config, &fil_config));
    // ESP_ERROR_CHECK(twai_start());
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    static httpd_handle_t server;
    server = start_webserver();

    xTaskCreatePinnedToCore(CAN_Receive_Process_Task, "TWAI_rx", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(CAN_Transmit_Process_Task, "TWAI_Tx", 4096, NULL, 3, NULL, 1);

    // ESP_ERROR_CHECK(twai_driver_install(&gen_config, &tim_config, &fil_config));
    // ESP_ERROR_CHECK(twai_start());
    // acc_pass = true;
    // count++;
   // xSemaphoreGive(ctrl_task_sem);
    
}
