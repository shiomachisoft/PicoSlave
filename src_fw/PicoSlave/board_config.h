#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// -----------------------------------------------------------------------------
// UART (Serial Communication) Configuration
// -----------------------------------------------------------------------------
#define UART_ID             uart0           // UART instance to use
#define UART_IRQ_NUM        UART0_IRQ       // UART IRQ number
#define UART_BAUDRATE       9600            // Baud rate
#define UART_TX_PIN         0               // TX pin (GP0)
#define UART_RX_PIN         1               // RX pin (GP1)
#define UART_DATA_BITS      8               // Data bits
#define UART_STOP_BITS      1               // Stop bits
#define UART_PARITY         UART_PARITY_NONE // Parity setting

// -----------------------------------------------------------------------------
// I2C (Slave Mode) Configuration
// -----------------------------------------------------------------------------
#define I2C_ID              i2c1            // I2C instance to use
#define I2C_BAUDRATE        100000          // Clock frequency (100 kHz)
#define I2C_SLAVE_ADDRESS   0x17            // Slave address
#define I2C_BUF_SIZE        256             // TX/RX buffer size
#define I2C_SDA_PIN         6               // SDA pin (GP6)
#define I2C_SCL_PIN         7               // SCL pin (GP7)

// -----------------------------------------------------------------------------
// SPI (Slave Mode) Configuration
// -----------------------------------------------------------------------------
#define SPI_RX_PIN          16              // RX (MISO) pin (GP16)
#define SPI_CSN_PIN         17              // CSn (Chip Select) pin (GP17)
#define SPI_SCK_PIN         18              // SCK pin (GP18)
#define SPI_TX_PIN          19              // TX (MOSI) pin (GP19)
#define SPI_CPOL            SPI_CPOL_1      // Clock polarity
#define SPI_CPHA            SPI_CPHA_1      // Clock phase
#define SPI_DATA_BITS       8               // Data bits
#define SPI_BUF_SIZE        256             // TX/RX buffer size

// -----------------------------------------------------------------------------
// PWM (Input Measurement) Configuration
// -----------------------------------------------------------------------------
#define PWM_MEASURE_PIN     21              // Measurement input pin (GP21)

#endif // BOARD_CONFIG_H