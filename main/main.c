/* smart mail

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
   * GPIO5:  output LED.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

// wifi
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

/*Set the SSID and Password via "make menuconfig"*/
#define DEFAULT_SSID CONFIG_WIFI_SSID
#define DEFAULT_PWD CONFIG_WIFI_PASSWORD

#if CONFIG_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_SCAN_METHOD*/

#if CONFIG_WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_WIFI_CONNECT_AP_BY_SECURITY
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_SORT_METHOD*/

#if CONFIG_FAST_SCAN_THRESHOLD
#define DEFAULT_RSSI CONFIG_FAST_SCAN_MINIMUM_SIGNAL
#if CONFIG_EXAMPLE_OPEN
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_WEP
#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_WPA
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_WPA2
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif /*CONFIG_FAST_SCAN_THRESHOLD*/

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

#define BLINK_GPIO CONFIG_BLINK_GPIO

#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}


// handler error for wifi
static const char *TAG = "scan";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGI(TAG, "Got IP: %s\n",
                     ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* Initialize Wi-Fi as sta and set scan method */
static void wifi_scan(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Zhone910491",
            .password = "3V4sT8gMrj",
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


//----------------Task blink_task------------------
void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
	printf("This is blink_task!\n");
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

//----------------Task print_sys_info------------------
void print_sys_info(void *pvParameter)
{
    /* Printing system information */
	esp_chip_info_t chip_info;
	printf("This is SmartMail!\n");
//	int i=0;
	while(1) {
		/* Print chip information */

		    esp_chip_info(&chip_info);
		    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
		            chip_info.cores,
		            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
		            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

		    printf("silicon revision %d, ", chip_info.revision);

		    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
		            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

		    for (int i = 10; i >= 0; i--) {
		        printf("Restarting in %d seconds...\n", i);
		        vTaskDelay(1000 / portTICK_PERIOD_MS);
		    }
		    printf("Restarting now.\n");
    }
}


//----------------Task connect_wifi------------------
void connect_wifi(void *pvParameter)
{
	// Initialize NVS
	    esp_err_t ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK( ret );

	    wifi_scan();
	while(1) {
		printf("while wifi\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




void app_main()
{
	gpio_config_t io_conf;

	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//change gpio intrrupt type for one pin
	gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

	//remove isr handler for gpio number.
	gpio_isr_handler_remove(GPIO_INPUT_IO_0);
	//hook isr handler for specific gpio pin again
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);


	xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&print_sys_info, "print_sys_info", 4095, NULL, 5, NULL);
    xTaskCreate(&connect_wifi, "connect_wifi", 8095, NULL, 5, NULL);

    int cnt = 0;
	while(1) {
		printf("cnt: %d\n", cnt++);
		vTaskDelay(1000 / portTICK_RATE_MS);

	}
}










//
////#######################################################################
///* Scan Example
//
//   This example code is in the Public Domain (or CC0 licensed, at your option.)
//
//   Unless required by applicable law or agreed to in writing, this
//   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//   CONDITIONS OF ANY KIND, either express or implied.
//*/
//
///*
//    This example shows how to use the All Channel Scan or Fast Scan to connect
//    to a Wi-Fi network.
//
//    In the Fast Scan mode, the scan will stop as soon as the first network matching
//    the SSID is found. In this mode, an application can set threshold for the
//    authentication mode and the Signal strength. Networks that do not meet the
//    threshold requirements will be ignored.
//
//    In the All Channel Scan mode, the scan will end only after all the channels
//    are scanned, and connection will start with the best network. The networks
//    can be sorted based on Authentication Mode or Signal Strength. The priority
//    for the Authentication mode is:  WPA2 > WPA > WEP > Open
//*/
//#include "freertos/FreeRTOS.h"
//#include "freertos/event_groups.h"
//#include "esp_wifi.h"
//#include "esp_log.h"
//#include "esp_event_loop.h"
//#include "nvs_flash.h"
//
///*Set the SSID and Password via "make menuconfig"*/
//#define DEFAULT_SSID CONFIG_WIFI_SSID
//#define DEFAULT_PWD CONFIG_WIFI_PASSWORD
//
//#if CONFIG_WIFI_ALL_CHANNEL_SCAN
//#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
//#elif CONFIG_WIFI_FAST_SCAN
//#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
//#else
//#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
//#endif /*CONFIG_SCAN_METHOD*/
//
//#if CONFIG_WIFI_CONNECT_AP_BY_SIGNAL
//#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
//#elif CONFIG_WIFI_CONNECT_AP_BY_SECURITY
//#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
//#else
//#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
//#endif /*CONFIG_SORT_METHOD*/
//
//#if CONFIG_FAST_SCAN_THRESHOLD
//#define DEFAULT_RSSI CONFIG_FAST_SCAN_MINIMUM_SIGNAL
//#if CONFIG_EXAMPLE_OPEN
//#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
//#elif CONFIG_EXAMPLE_WEP
//#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
//#elif CONFIG_EXAMPLE_WPA
//#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
//#elif CONFIG_EXAMPLE_WPA2
//#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
//#else
//#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
//#endif
//#else
//#define DEFAULT_RSSI -127
//#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
//#endif /*CONFIG_FAST_SCAN_THRESHOLD*/
//
//static const char *TAG = "scan";
//
//static esp_err_t event_handler(void *ctx, system_event_t *event)
//{
//    switch (event->event_id) {
//        case SYSTEM_EVENT_STA_START:
//            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
//            ESP_ERROR_CHECK(esp_wifi_connect());
//            break;
//        case SYSTEM_EVENT_STA_GOT_IP:
//            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
//            ESP_LOGI(TAG, "Got IP: %s\n",
//                     ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
//            break;
//        case SYSTEM_EVENT_STA_DISCONNECTED:
//            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
//            ESP_ERROR_CHECK(esp_wifi_connect());
//            break;
//        default:
//            break;
//    }
//    return ESP_OK;
//}
//
///* Initialize Wi-Fi as sta and set scan method */
//static void wifi_scan(void)
//{
//    tcpip_adapter_init();
//    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
//
//    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//    wifi_config_t wifi_config = {
//        .sta = {
//            .ssid = DEFAULT_SSID,
//            .password = DEFAULT_PWD,
//            .scan_method = DEFAULT_SCAN_METHOD,
//            .sort_method = DEFAULT_SORT_METHOD,
//            .threshold.rssi = DEFAULT_RSSI,
//            .threshold.authmode = DEFAULT_AUTHMODE,
//        },
//    };
//    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
//    ESP_ERROR_CHECK(esp_wifi_start());
//}
//
//void app_main()
//{
//    // Initialize NVS
//    esp_err_t ret = nvs_flash_init();
//    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
//        ESP_ERROR_CHECK(nvs_flash_erase());
//        ret = nvs_flash_init();
//    }
//    ESP_ERROR_CHECK( ret );
//
//    wifi_scan();
//}
