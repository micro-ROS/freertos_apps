#include <uxr/client/profile/transport/serial/serial_transport_external.h>

#include <driver/uart.h>
#include <driver/gpio.h>

#define UART_TXD  (GPIO_NUM_4)
#define UART_RXD  (GPIO_NUM_5)
#define UART_RTS  (UART_PIN_NO_CHANGE)
#define UART_CTS  (UART_PIN_NO_CHANGE)

#define UART_BUFFER_SIZE (1024)


bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    if (fd != 1) {
        return false;
    }

    platform->uart_port = UART_NUM_1;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(platform->uart_port, &uart_config) == ESP_FAIL) {
        return false;
    }
    if (uart_set_pin(platform->uart_port, UART_TXD, UART_RXD, UART_RTS, UART_CTS) == ESP_FAIL) {
        return false;
    }
    if (uart_driver_install(platform->uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL) {
        return false;
    }

    return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{   
    return uart_driver_delete(platform->uart_port) == ESP_OK;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
    const int txBytes = uart_write_bytes(platform->uart_port, buf, len);
    return txBytes;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{ 
    const int rxBytes = uart_read_bytes(platform->uart_port, buf, len, timeout / portTICK_RATE_MS);
    return rxBytes;
}

