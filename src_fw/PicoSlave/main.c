#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/i2c_slave.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "board_config.h"

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------

// For I2C Communication
volatile int g_i2c_buffer_index = 0;                // Current position in I2C buffer
volatile uint8_t g_i2c_buffer[I2C_BUF_SIZE];        // I2C data TX/RX buffer

// For SPI Communication (Used in DMA transfer)
uint8_t g_spi_tx_buffer[SPI_BUF_SIZE];              // SPI TX data buffer
uint8_t g_spi_rx_buffer[SPI_BUF_SIZE];              // SPI RX data buffer

// -----------------------------------------------------------------------------
// Interrupt Handlers / Callbacks
// -----------------------------------------------------------------------------

// I2C Slave Event Handler
// Handles write/read requests from master.
void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) 
{
    switch (event) {
    case I2C_SLAVE_RECEIVE: // Data received from master (Write)
        if (g_i2c_buffer_index < sizeof(g_i2c_buffer)) {
            g_i2c_buffer[g_i2c_buffer_index] = i2c_read_byte_raw(i2c);
            g_i2c_buffer_index++;
        }
        break;

    case I2C_SLAVE_REQUEST: // Data transmit to master (Read request)
        if (g_i2c_buffer_index < sizeof(g_i2c_buffer)) {
            i2c_write_byte_raw(i2c, g_i2c_buffer[g_i2c_buffer_index]);
            g_i2c_buffer_index++;
        }
        break;

    case I2C_SLAVE_FINISH: // Transaction finished
        g_i2c_buffer_index = 0;
        break;
    default:
        break;
    }
}

// UART RX Interrupt Handler
// Echoes back received data.
void uart_rx_handler() 
{
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        // Echo back received character
        if (uart_is_writable(UART_ID)) {
            uart_putc(UART_ID, ch);
        }
    }
}

// -----------------------------------------------------------------------------
// Hardware Setup / Helper Functions
// -----------------------------------------------------------------------------

// Measure duty cycle from PWM input
// Measures high period of specified GPIO pin (PWM B channel required).
// Args: gpio GPIO pin number to measure
// Returns: float Duty cycle (0.0 - 1.0)
float pwm_measure_duty_cycle(uint gpio) 
{
    // Only PWM B channel pins can be used as input
    assert(pwm_gpio_to_channel(gpio) == PWM_CHAN_B);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Count once every 100 cycles while PWM B input is High
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
    pwm_config_set_clkdiv(&cfg, 100);
    pwm_init(slice_num, &cfg, false);
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Start measurement (for 10ms)
    pwm_set_enabled(slice_num, true);
    sleep_ms(10);
    pwm_set_enabled(slice_num, false);

    // Calculate duty cycle from count value
    float counting_rate = clock_get_hz(clk_sys) / 100;
    float max_possible_count = counting_rate * 0.01; // 0.01s = 10ms
    return pwm_get_counter(slice_num) / max_possible_count;
}

// Initialize I2C Slave
void i2c_slave_setup() 
{
    memset((void*)g_i2c_buffer, 0, sizeof(g_i2c_buffer));

    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_init(I2C_ID, I2C_BAUDRATE);
    
    // Configure I2C slave mode and register handler
    i2c_slave_init(I2C_ID, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

// Initialize SPI Slave
void spi_slave_setup() 
{
    // Baud rate is ignored in slave mode, but required by API
    spi_init(spi_default, 1 * 1000000);
    
    // Disable SPI temporarily for configuration change
    hw_clear_bits(&spi_get_hw(spi_default)->cr1, SPI_SSPCR1_SSE_BITS);
    
    spi_set_format(spi_default, SPI_DATA_BITS, SPI_CPOL, SPI_CPHA, SPI_MSB_FIRST);
    spi_set_slave(spi_default, true);
    
    // Re-enable SPI
    hw_set_bits(&spi_get_hw(spi_default)->cr1, SPI_SSPCR1_SSE_BITS);

    gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CSN_PIN, GPIO_FUNC_SPI); 

    // Initialize TX buffer (Test data: bitwise NOT value)
    for (int i = 0; i < SPI_BUF_SIZE; i++) {
        g_spi_tx_buffer[i] = ~i;
    } 
}

// -----------------------------------------------------------------------------
// Core 1 Main Process
// -----------------------------------------------------------------------------

// Core 1 Entry Point
// Controls DMA transfer for SPI communication.
void core1_main() 
{
    dma_channel_config dma_config;
    uint dma_tx_channel; // DMA channel for TX
    uint dma_rx_channel; // DMA channel for RX

    // Claim unused DMA channels
    dma_tx_channel = dma_claim_unused_channel(true);
    dma_rx_channel = dma_claim_unused_channel(true); 

    while (1) {
        // --- DMA Configuration for TX ---
        // Transfer from memory (g_spi_tx_buffer) to SPI data register (DR)
        dma_config = dma_channel_get_default_config(dma_tx_channel);
        channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
        channel_config_set_dreq(&dma_config, spi_get_dreq(spi_default, true));
        channel_config_set_read_increment(&dma_config, true);  // Read address increments
        channel_config_set_write_increment(&dma_config, false); // Write address (SPI DR) is fixed
        dma_channel_configure(dma_tx_channel, &dma_config,
                            &spi_get_hw(spi_default)->dr, // Write address
                            g_spi_tx_buffer,              // Read address
                            SPI_BUF_SIZE,                 // Transfer size
                            false);                       // Do not start yet

        // --- DMA Configuration for RX ---
        // Transfer from SPI data register (DR) to memory (g_spi_rx_buffer)
        dma_config = dma_channel_get_default_config(dma_rx_channel);
        channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
        channel_config_set_dreq(&dma_config, spi_get_dreq(spi_default, false));
        channel_config_set_read_increment(&dma_config, false); // Read address (SPI DR) is fixed
        channel_config_set_write_increment(&dma_config, true); // Write address increments
        dma_channel_configure(dma_rx_channel, &dma_config,
                            g_spi_rx_buffer,              // Write address
                            &spi_get_hw(spi_default)->dr, // Read address
                            SPI_BUF_SIZE,                 // Transfer size
                            false);                       // Do not start yet

        // Start TX and RX DMA simultaneously
        dma_start_channel_mask((1u << dma_tx_channel) | (1u << dma_rx_channel));
        
        // Wait for TX completion (blocking)
        dma_channel_wait_for_finish_blocking(dma_tx_channel);     
    }
}

// -----------------------------------------------------------------------------
// Core 0 Main Process
// -----------------------------------------------------------------------------
int main() 
{
    stdio_init_all();
    
    // Initialize UART
    uart_init(UART_ID, UART_BAUDRATE);  // Baud rate

    // Set pin functions
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);

    // Communication settings
    uart_set_format(UART_ID, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
    // Disable UART FIFO to match sample program
    uart_set_fifo_enabled(UART_ID, false);

    // Setup UART RX interrupt
    int uart_irq = UART_IRQ_NUM;
    irq_set_exclusive_handler(uart_irq, uart_rx_handler);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(UART_ID, true, false);

    // Setup I2C Slave
    i2c_slave_setup();

    // Setup SPI
    spi_slave_setup();

    // Wait for stability after startup
    busy_wait_ms(100);

    // Launch Core 1 main loop
    multicore_launch_core1(core1_main); 

    while (1) {
        // Display PWM input measurement result
        float measured_duty_cycle = pwm_measure_duty_cycle(PWM_MEASURE_PIN);
        printf("measured input duty cycle = %.1f%%\n",
                measured_duty_cycle * 100.f); 
        sleep_ms(5000);              
    }
}
