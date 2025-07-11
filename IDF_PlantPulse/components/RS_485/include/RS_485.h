
#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Modbus Configuration
#define MB_BUFFER_SIZE     256               // Modbus buffer size
// #define MB_SLAVE_ADDR      0x03              // Modbus slave address
// #define MB_FUNC_CODE       0x03              // Modbus function code (read holding registers)



// Modbus RTU Configuration
#define MB_UART_PORT UART_NUM_2  // UART port for Modbus RTU
#define MB_BAUD_RATE 9600        // Baud rate for Modbus RTU
#define MB_DATA_BITS UART_DATA_8_BITS
#define MB_PARITY UART_PARITY_DISABLE
#define MB_STOP_BITS UART_STOP_BITS_1
#define MB_RX_PIN 17             // GPIO pin for RX
#define MB_TX_PIN 18             // GPIO pin for TX
#define MB_RTS_PIN UART_PIN_NO_CHANGE
#define MB_CTS_PIN UART_PIN_NO_CHANGE
#define MB_RX_BUFFER_SIZE 256    // RX buffer size
#define MB_TX_BUFFER_SIZE 256    // TX buffer size

// Modbus RTU Function Codes
// #define MB_READ_COILS 0x01
// #define MB_READ_DISCRETE_INPUTS 0x02
// #define MB_READ_HOLDING_REGISTERS 0x03
// #define MB_READ_INPUT_REGISTERS 0x04
// #define MB_WRITE_SINGLE_COIL 0x05
// #define MB_WRITE_SINGLE_REGISTER 0x06
// #define MB_WRITE_MULTIPLE_COILS 0x0F
// #define MB_WRITE_MULTIPLE_REGISTERS 0x10
// Modbus RTU Exceptions
// #define MB_EXCEPTION_ILLEGAL_FUNCTION 0x01
// #define MB_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
// #define MB_EXCEPTION_ILLEGAL_DATA_VALUE 0x03

// Function Prototypes
void PH_init(void);
void MB_rtu_send(uint8_t slave_address,
                 uint8_t function_code, 
                 uint16_t start_address, 
                 uint16_t quantity, 
                 uint8_t *data, 
                 uint16_t data_length);

uint8_t MB_rtu_receive(uint8_t *response_buffer, uint16_t buffer_size);
uint16_t crc16(uint8_t *data, uint16_t length);



