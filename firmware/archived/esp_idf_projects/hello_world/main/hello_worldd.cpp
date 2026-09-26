#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"

#include "data_structs.h"

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR

// esp_log_level_set("*", ESP_LOG_ERROR);        // set all components to ERROR level

static const char *TAG = "example";

const auto PIN_LED_PWM = GPIO_NUM_2;

#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100

// #define EXAMPLE_EC11_GPIO_A 0
// #define EXAMPLE_EC11_GPIO_B 2

const auto PIN_ENCODER1A = GPIO_NUM_11;
const auto PIN_ENCODER1B = GPIO_NUM_12;

#define BUF_SIZE (1024)

DataPacket_t dataPacket;
// dataPacket.state.motor1EncoderPulses = 0;



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
        
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));

        uart_write_bytes(UART_PORT_NUM, &STX, 1);
        uart_write_bytes(UART_PORT_NUM, (uint8_t *) &packetHeader, sizeof( packetHeader ));
        uart_write_bytes(UART_PORT_NUM, (uint8_t *) &dataPacket, sizeof( dataPacket ));
        uart_write_bytes(UART_PORT_NUM, &ETX, 1);

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // delay 1 sec
    }
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

static void pulse_cnt_task(void *arg)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = PIN_ENCODER1A,
        .level_gpio_num = PIN_ENCODER1B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = PIN_ENCODER1B,
        .level_gpio_num = PIN_ENCODER1A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // Report counter value
    int pulse_count = 0;
    int event_count = 0;
    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
            ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        } else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        }
    }
}


extern "C"{
void app_main();
}


    
void app_main(void)
{
    

    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);

    xTaskCreate(pulse_cnt_task, "pulse_count_task", 2048, NULL, 10, NULL);


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
