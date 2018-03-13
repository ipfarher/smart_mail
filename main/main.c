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

    int cnt = 0;
	while(1) {
		printf("cnt: %d\n", cnt++);
		vTaskDelay(1000 / portTICK_RATE_MS);

	}
}
