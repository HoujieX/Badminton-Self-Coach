#ifndef ST7735_H_
#define ST7735_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"
#include "sl_driver_gpio.h"
#include "sl_si91x_driver_gpio.h"

#define ST7735_SCK_GPIO_NUM   8U
#define ST7735_MOSI_GPIO_NUM  9U
#define ST7735_CS_GPIO_NUM    15U
#define ST7735_RST_GPIO_NUM   25U
#define ST7735_DC_GPIO_NUM    26U

#define ST7735_SCK_PORT       SL_GPIO_PORT_D
#define ST7735_MOSI_PORT      SL_GPIO_PORT_D
#define ST7735_CS_PORT        SL_GPIO_PORT_A
#define ST7735_RST_PORT       SL_GPIO_PORT_A
#define ST7735_DC_PORT        SL_GPIO_PORT_A

#define ST7735_DC_COMMAND_LEVEL 0U
#define ST7735_DC_DATA_LEVEL    1U

#define LCD_WIDTH 160
#define LCD_HEIGHT 128
#define LCD_SIZE  LCD_WIDTH * LCD_HEIGHT

// If only part of the panel updates, tune these offsets (common ST7735R values: 0/0 or 2/1).
#define ST7735_X_OFFSET 0U
#define ST7735_Y_OFFSET 0U

//! \name Return error codes
//! @{
#define ADAFRUIT358_SPI_NO_ERR                 0 //! No error
#define ADAFRUIT358_SPI_ERR                    1 //! General or an unknown error
#define ADAFRUIT358_SPI_ERR_RESP_TIMEOUT       2 //! Timeout during command
#define ADAFRUIT358_SPI_ERR_RESP_BUSY_TIMEOUT  3 //! Timeout on busy signal of R1B response
#define ADAFRUIT358_SPI_ERR_READ_TIMEOUT       4 //! Timeout during read operation
#define ADAFRUIT358_SPI_ERR_WRITE_TIMEOUT      5 //! Timeout during write operation
#define ADAFRUIT358_SPI_ERR_RESP_CRC           6 //! Command CRC error
#define ADAFRUIT358_SPI_ERR_READ_CRC           7 //! CRC error during read operation
#define ADAFRUIT358_SPI_ERR_WRITE_CRC          8 //! CRC error during write operation
#define ADAFRUIT358_SPI_ERR_ILLEGAL_COMMAND    9 //! Command not supported
#define ADAFRUIT358_SPI_ERR_WRITE             10 //! Error during write operation
#define ADAFRUIT358_SPI_ERR_OUT_OF_RANGE      11 //! Data access out of range
//! @}

// ST7735 registers
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09
#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E
#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD
#define ST7735_PWCTR6  0xFC
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Delay_ms(unsigned int n);
void lcd_init(void);
void sendCommands (const uint8_t *cmds, uint8_t length);
void LCD_setAddr(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void SPI_ControllerTx(uint8_t data);
void SPI_ControllerTx_stream(uint8_t stream);
void SPI_ControllerTx_16bit(uint16_t data);
void SPI_ControllerTx_16bit_stream(uint16_t data);
void LCD_brightness(uint8_t intensity);
void LCD_rotate(uint8_t r);

#endif /* ST7735_H_ */
