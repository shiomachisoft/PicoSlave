# PicoSlave

## Functional Overview

This firmware operates the Raspberry Pi Pico as a slave (UART/I2C/SPI/PWM) for verifying communication continuity.

*   **UART (Serial Communication)**
    *   Provides an "echo back" function that sends received data back as is.
*   **I2C Slave**
    *   When the master writes data, the received data is sequentially saved to an internal buffer (256 bytes). Subsequently, when the master reads, the data previously written by the master is returned. (e.g., if the master writes `0x10`, `0x20`, reading will return `0x10`, `0x20` ... in that order).
*   **SPI Slave**
    *   When data is sent from the master, the slave simultaneously sends back data (fixed test values in this firmware) synchronized with the clock sent from the master. Specifically, values are output in the order of `0xFF`, `0xFE`, `0xFD` ... `0x00`.
*   **PWM (Pulse Width Modulation) Measurement**
    *   Measures the duty cycle (percentage of High duration) of the PWM signal input to the input pin and outputs the log to a PC via USB serial (virtual COM port).

## Wiring (Pinout)

The default pin configuration is defined in `board_config.h`.

| Interface | Pin (GP) | Signal Line |
| :--- | :--- | :--- |
| **UART** | GP0 | TX |
| | GP1 | RX |
| **I2C** | GP6 | SDA |
| | GP7 | SCL |
| **SPI** | GP16 | RX |
| | GP17 | CSn |
| | GP18 | SCK |
| | GP19 | TX |
| **PWM** | GP21 | Input |

## Communication Settings

The default settings for each function are as follows.

**If you want to change these settings or pin assignments, please edit `board_config.h`.**


### UART
*   **Baud Rate**: 9600 bps
*   **Data Format**: 8 data bits, 1 stop bit, no parity

### I2C Slave
*   **Slave Address**: `0x17`
*   **Speed**: 100 kHz
*   **Buffer Size**: 256 bytes

### SPI Slave
*   **Bit Order**: MSB First (Fixed)
*   **Data Bit Length**: 8 bit
*   **Buffer Size**: 256 bytes
*   **Clock Polarity/Phase**: CPOL=1, CPHA=1

     *   **Important**: Please keep CPHA=1. (Do not change to CPHA=0.)
     *   **Reason**: While standard SPI allows sending multiple bytes continuously by keeping CS Low and sending clocks (SCLK), the RP2040 (Pico) SPI slave has the following specific limitation:
         * "Per-frame" CS toggle requirement (CPHA=0):
     Due to the specification of the SPI controller on the RP2040, when operating in SPI mode 0 or 2 (CPHA=0), the master must return CS to High once after sending 1 data unit (1 byte or 1 word) for the next data to be received correctly (or to maintain synchronization).
         * Standard expectation: Receive 10 bytes by sending 8 clocks x 10 times while keeping CS Low.
         * Pico (Slave) reality: If clocks continue while CS is Low, it may fail to recognize the end timing of the 1st data correctly, causing sampling shifts for the 2nd data onwards or failure to store in FIFO properly.

<img width="789" height="212" alt="image" src="https://github.com/user-attachments/assets/0ded836d-8784-46d6-9e45-81a67db0db69" />

### PWM
*   **Function**: Input signal duty cycle measurement

## Usage

1.  First, perform the necessary wiring according to the function you want to use. Refer to the "Wiring (Pinout)" chapter for pin assignments.
2.  Copy the `PicoSlave.uf2` file to the Pico (RPI-RP2 drive) connected to the PC via USB while holding down the BOOTSEL button.
    ⇒ After copying is complete, the Pico will automatically reboot and this firmware will start operating.
3.  Open the Pico's COM port with serial monitor software such as TeraTerm, and the PWM measurement result log will be displayed at intervals of about 5 seconds. (Set the TeraTerm baud rate to 115200 bps for now.)
4.  Send data from the master device of each interface (UART, I2C, SPI) to PicoSlave to check the operation. Refer to "Functional Overview" for the content of the data returned by PicoSlave.

## Memo
> [!TIP]
> The following configuration in `CMakeLists.txt` ensures the entire program is loaded into RAM for high-speed execution:
>
> ```cmake
> pico_set_binary_type(PicoSlave copy_to_ram)
> ```

## Disclaimer
The author assumes no responsibility for any damages or troubles arising from the use of this software. Please use it at your own risk.

