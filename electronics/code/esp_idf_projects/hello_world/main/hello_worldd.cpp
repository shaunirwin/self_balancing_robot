#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"

#include "data_structs.h"

static const char *TAG = "example";

const auto PIN_LED_PWM = GPIO_NUM_2;

#define BUF_SIZE (1024)



static void echo_task(void *arg)
{
    const auto UART_PORT_NUM = UART_NUM_1;
    const int PIN_UART_TXD = 43;
    const int PIN_UART_RXD = 44;
    // const int PIN_UART_RTS = ;
    // const int PIN_UART_CTS;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh=122,
        .source_clk = UART_SCLK_DEFAULT,
        .flags=0
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_UART_TXD, PIN_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    while (1) {
        PacketHeader_t packetHeader;
        packetHeader.packetID = 268;
        packetHeader.microSecondsSinceBoot = esp_timer_get_time();
        
        DataPacket_t dataPacket;

        uart_write_bytes(UART_PORT_NUM, &STX, 1);
        uart_write_bytes(UART_PORT_NUM, (uint8_t *) &packetHeader, sizeof( packetHeader ));
        uart_write_bytes(UART_PORT_NUM, (uint8_t *) &dataPacket, sizeof( dataPacket ));
        uart_write_bytes(UART_PORT_NUM, &ETX, 1);

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // delay 1 sec
    }
}

extern "C"{
void app_main();
}


    
void app_main(void)
{
    

    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);


    /* Reset the pin */
    gpio_reset_pin(PIN_LED_PWM);
    /* Set the GPIOs to Output mode */
    gpio_set_direction(PIN_LED_PWM, GPIO_MODE_OUTPUT);
    while (1) 
    {
        gpio_set_level(PIN_LED_PWM, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Turning the LED %s!","ON");
        gpio_set_level(PIN_LED_PWM, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Turning the LED %s!","OFF");
    }
}
