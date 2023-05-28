#include <arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */


void uart_init(int baudRate, int tx_io_num, int rx_io_num, uart_port_t uart_num)
{
    #define BUF_SIZE 1024

    #if CONFIG_UART_ISR_IN_IRAM
      intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    uart_config_t uart_config = 
    {
      .baud_rate = baudRate,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
   /* Configure parameters of an UART driver,
   * communication pins and install the driver */  
   int intr_alloc_flags = 0;
   uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
   uart_param_config(uart_num, &uart_config);
   uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}

char* uartData()
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
  
    if (len > 0) {
      data[len] = 0;
      uart_write_bytes(UART_NUM_0, (const char *) data, len);
      char* output = (char *) data;
      free(data);
      return output;
    }

    return nullptr;
}




void setup()
{
  uart_init(115200, GPIO_NUM_1, GPIO_NUM_3, UART_NUM_0);
  pinMode(13,OUTPUT);
  pinMode(14,OUTPUT);
}

void loop()
{
  
  //char *teste = uartData();

  digitalWrite(13,HIGH);
  digitalWrite(14,HIGH);

  vTaskDelay(100 / portTICK_PERIOD_MS);
}
