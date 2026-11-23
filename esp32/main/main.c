#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "../inc/wifi_sta.h"
#include "../inc/mqtt.h"

#define FRAME_HEADER_1  0xAA
#define FRAME_HEADER_2  0x55
#define PACKET_SIZE 9

static QueueHandle_t uart_queue = NULL;
static QueueHandle_t system_data_queue = NULL;
static unsigned int err_f_data_c = 0;
static unsigned int err_ovf_c = 0;

typedef enum 
{
    WAIT_HEADER_1 = 0,
    WAIT_HEADER_2,
    READ_PAYLOAD
} parser_state_t;

typedef struct
{
    parser_state_t state;
    int payload_idx;
    uint8_t raw_data[PACKET_SIZE];
} parser_t;

typedef struct __attribute__((packed))
{
    int16_t temp;
    uint32_t press;
    uint8_t hum;
    uint8_t motion;
} system_data_t;

void uart_rx_task(void *pvParameters);
void mqtt_publish_task(void *pvParameters);
void config_uart(void);
void parse_and_transfer_data(parser_t *parser, uint8_t *data, int rx_bytes);
int calc_cmp_crc8(uint8_t *data, size_t length, uint8_t rx_crc);

void app_main(void)
{
    system_data_queue = xQueueCreate(10, sizeof(system_data_t));
    configASSERT(system_data_queue != NULL);
    
    config_uart();
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    wifi_init_sta();
    mqtt_app_start();

    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 4, NULL);
}

void uart_rx_task(void *pvParameters)
{
    (void)pvParameters;

    uart_event_t event = {0};
    parser_t parser = {0};
    uint8_t buf[128];
    int rx_bytes = 0;

    for (;;)
    {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE) continue;
        if (event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR) 
        {
            err_f_data_c++;
            ESP_LOGW("ERR_DATA", "Received false data (%u)", err_f_data_c);
            continue;
        }
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL)
        {
            uart_flush_input(UART_NUM_2);
            xQueueReset(uart_queue);
            memset(&parser, 0, sizeof(parser));
            err_ovf_c++;
            ESP_LOGW("ERR_OVF", "Fifo or buffer overflow (%u)", err_ovf_c);
            continue;
        }
        if (event.type != UART_DATA) continue;

        size_t to_read = (event.size < sizeof(buf)) ? event.size : sizeof(buf);
        rx_bytes = uart_read_bytes(UART_NUM_2, buf, to_read, pdMS_TO_TICKS(100));
        if (rx_bytes > 0)
        {
            parse_and_transfer_data(&parser, buf, rx_bytes);
        }
    }
}

void mqtt_publish_task(void *pvParameters)
{
    (void)pvParameters;
    system_data_t rx_data = {0};
    char json_buf[64];

    for(;;)
    {
        if (xQueueReceive(system_data_queue, &rx_data, portMAX_DELAY) != pdTRUE) continue;
        snprintf(json_buf, sizeof(json_buf), "{\"temp\":%.2f,\"press\":%lu,\"hum\":%u,\"motion\":%u}",
                                rx_data.temp / 100.0f, (unsigned long)rx_data.press/25600, rx_data.hum, rx_data.motion);
        mqtt_publish_telemetry(json_buf);
    }
}

void config_uart(void)
{
    uart_config_t uart_config = {0};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024, 0, 10, &uart_queue, 0);
}

void parse_and_transfer_data(parser_t *parser, uint8_t *data, int rx_bytes)
{
    system_data_t system_data = {0};

    for (int i = 0; i < rx_bytes; i++)
    {
        switch (parser->state)
        {
            case WAIT_HEADER_1:
                if (data[i] == FRAME_HEADER_1)
                {
                    parser->state = WAIT_HEADER_2;
                }
                break;

            case WAIT_HEADER_2:
                if (data[i] == FRAME_HEADER_2)
                {
                    parser->state = READ_PAYLOAD;
                }
                else if (data[i] != FRAME_HEADER_1)
                {
                    parser->state = WAIT_HEADER_1;
                }
                break;
            
            case READ_PAYLOAD:
                if ((size_t)parser->payload_idx < sizeof(parser->raw_data))
                {
                    parser->raw_data[parser->payload_idx] = data[i];
                    parser->payload_idx++;
                }
                if ((size_t)parser->payload_idx >= sizeof(parser->raw_data)) 
                {
                    parser->payload_idx = 0;
                    parser->state = WAIT_HEADER_1;
                    if (calc_cmp_crc8(parser->raw_data, PACKET_SIZE - 1, parser->raw_data[PACKET_SIZE - 1]))
                    {
                        memcpy((uint8_t *)&system_data, parser->raw_data, PACKET_SIZE - 1);
                        if (xQueueSend(system_data_queue, &system_data, pdMS_TO_TICKS(0)) != pdTRUE)
                        {
                            ESP_LOGW("ERR_QUEUE", "system_data_queue is full, dropping packet");
                        }
                    }
                }
                break;
            
            default:
                parser->payload_idx = 0;
                parser->state = WAIT_HEADER_1;
                break;
        }
    }
}

int calc_cmp_crc8(uint8_t *data, size_t length, uint8_t rx_crc)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
        }
    }
    return (crc == rx_crc);
}