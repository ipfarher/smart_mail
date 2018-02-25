/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

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



void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&print_sys_info, "print_sys_info", 4095, NULL, 5, NULL);
}
