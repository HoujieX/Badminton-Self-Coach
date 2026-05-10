#include "ST7735.h"

#include <stddef.h>
#define ST7735_SPI_DELAY_MS 0U

static const sl_gpio_t st7735_sck_gpio  = { ST7735_SCK_PORT, ST7735_SCK_GPIO_NUM };
static const sl_gpio_t st7735_mosi_gpio = { ST7735_MOSI_PORT, ST7735_MOSI_GPIO_NUM };
static const sl_gpio_t st7735_cs_gpio   = { ST7735_CS_PORT, ST7735_CS_GPIO_NUM };
static const sl_gpio_t st7735_rst_gpio  = { ST7735_RST_PORT, ST7735_RST_GPIO_NUM };
static const sl_gpio_t st7735_dc_gpio   = { ST7735_DC_PORT, ST7735_DC_GPIO_NUM };
static bool st7735_gpio_driver_initialized = false;

static void st7735_gpio_driver_init_once(void)
{
  if (!st7735_gpio_driver_initialized) {
    sl_status_t status = sl_gpio_driver_init();
    st7735_gpio_driver_initialized = (status == SL_STATUS_OK);
  }
}

static void st7735_gpio_write(const sl_gpio_t *gpio, bool high);

static void st7735_gpio_write(const sl_gpio_t *gpio, bool high)
{
  if (high) {
    (void)sl_gpio_driver_set_pin((sl_gpio_t *)gpio);
  } else {
    (void)sl_gpio_driver_clear_pin((sl_gpio_t *)gpio);
  }
}

static void st7735_spi_delay(void)
{
  if (ST7735_SPI_DELAY_MS > 0U) {
    osDelay(ST7735_SPI_DELAY_MS);
  } else {
    volatile uint32_t spin = 4U;
    while (spin-- > 0U) {
    }
  }
}

static void st7735_gpio_init_pin(const sl_gpio_t *gpio)
{
  sl_si91x_gpio_pin_config_t pin_config = { { gpio->port, gpio->pin }, GPIO_OUTPUT };

  (void)sl_gpio_set_configuration(pin_config);
  st7735_gpio_write(gpio, false);
}

static void lcd_pin_init(void)
{
  st7735_gpio_driver_init_once();

  st7735_gpio_init_pin(&st7735_sck_gpio);
  st7735_gpio_init_pin(&st7735_mosi_gpio);
  st7735_gpio_init_pin(&st7735_cs_gpio);
  st7735_gpio_init_pin(&st7735_rst_gpio);
  st7735_gpio_init_pin(&st7735_dc_gpio);

  st7735_gpio_write(&st7735_cs_gpio, false);
  st7735_gpio_write(&st7735_sck_gpio, false);
  st7735_gpio_write(&st7735_mosi_gpio, false);
  st7735_gpio_write(&st7735_dc_gpio, (ST7735_DC_DATA_LEVEL != 0U));

  Delay_ms(50U);
  st7735_gpio_write(&st7735_rst_gpio, true);
  Delay_ms(50U);
}

void Delay_ms(unsigned int n)
{
  if (n == 0U) {
    return;
  }

  if (osKernelGetState() == osKernelRunning) {
    osDelay(n);
    return;
  }

  while (n-- > 0U) {
    volatile uint32_t spin = 12000U;
    while (spin-- > 0U) {
    }
  }
}

void SPI_ControllerTx_stream(uint8_t stream)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
    st7735_gpio_write(&st7735_mosi_gpio, (stream & mask) != 0U);
    st7735_spi_delay();
    st7735_gpio_write(&st7735_sck_gpio, true);
    st7735_spi_delay();
    st7735_gpio_write(&st7735_sck_gpio, false);
  }
}

void SPI_ControllerTx(uint8_t data)
{
  SPI_ControllerTx_stream(data);
}

void SPI_ControllerTx_16bit_stream(uint16_t data)
{
  SPI_ControllerTx_stream((uint8_t)(data >> 8));
  SPI_ControllerTx_stream((uint8_t)(data & 0xFFU));
}

void SPI_ControllerTx_16bit(uint16_t data)
{
  SPI_ControllerTx_16bit_stream(data);
}

void lcd_init(void)
{
  lcd_pin_init();
  Delay_ms(5U);

  static const uint8_t st7735_cmds[] = {
    ST7735_SWRESET, 0, 150,
    ST7735_SLPOUT, 0, 255,
    ST7735_FRMCTR1, 3, 0x01, 0x2C, 0x2D, 0,
    ST7735_FRMCTR2, 3, 0x01, 0x2C, 0x2D, 0,
    ST7735_FRMCTR3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, 0,
    ST7735_INVCTR, 1, 0x07, 0,
    ST7735_PWCTR1, 3, 0x0A, 0x02, 0x84, 5,
    ST7735_PWCTR2, 1, 0xC5, 5,
    ST7735_PWCTR3, 2, 0x0A, 0x00, 5,
    ST7735_PWCTR4, 2, 0x8A, 0x2A, 5,
    ST7735_PWCTR5, 2, 0x8A, 0xEE, 5,
    ST7735_VMCTR1, 1, 0x0E, 0,
    ST7735_INVOFF, 0, 0,
    ST7735_MADCTL, 1, 0xC8, 0,
    ST7735_COLMOD, 1, 0x05, 0,
    ST7735_CASET, 4, 0x00, ST7735_X_OFFSET, 0x00,
    (uint8_t)(ST7735_X_OFFSET + LCD_WIDTH - 1U), 0,
    ST7735_RASET, 4, 0x00, ST7735_Y_OFFSET, 0x00,
    (uint8_t)(ST7735_Y_OFFSET + LCD_HEIGHT - 1U), 0,
    ST7735_GMCTRP1, 16, 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
    0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10, 0,
    ST7735_GMCTRN1, 16, 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
    0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10, 0,
    ST7735_NORON, 0, 10,
    ST7735_DISPON, 0, 100,
    ST7735_MADCTL, 1, MADCTL_MX | MADCTL_MV | MADCTL_RGB, 10
  };
  
  sendCommands(st7735_cmds, 22U);
}

void sendCommands(const uint8_t *cmds, uint8_t length)
{
  uint8_t num_commands = length;

  while (num_commands-- != 0U) {
    uint8_t num_data;
    uint8_t wait_time;

    st7735_gpio_write(&st7735_dc_gpio, (ST7735_DC_COMMAND_LEVEL != 0U));
    SPI_ControllerTx_stream(*cmds++);

    num_data = *cmds++;

    st7735_gpio_write(&st7735_dc_gpio, (ST7735_DC_DATA_LEVEL != 0U));
    while (num_data-- != 0U) {
      SPI_ControllerTx_stream(*cmds++);
    }

    wait_time = *cmds++;
    if (wait_time != 0U) {
      Delay_ms((wait_time == 255U) ? 500U : wait_time);
    }
  }

}

void LCD_setAddr(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  uint8_t x0_off = (uint8_t)(x0 + ST7735_X_OFFSET);
  uint8_t x1_off = (uint8_t)(x1 + ST7735_X_OFFSET);
  uint8_t y0_off = (uint8_t)(y0 + ST7735_Y_OFFSET);
  uint8_t y1_off = (uint8_t)(y1 + ST7735_Y_OFFSET);

  uint8_t st7735_cmds[] = {
    ST7735_CASET, 4, 0x00, x0_off, 0x00, x1_off, 0,
    ST7735_RASET, 4, 0x00, y0_off, 0x00, y1_off, 0,
    ST7735_RAMWR, 0, 5
  };

  sendCommands(st7735_cmds, 3U);
}

void LCD_brightness(uint8_t intensity)
{
  (void)intensity;
}

void LCD_rotate(uint8_t r)
{
  uint8_t madctl   = 0U;
  uint8_t rotation = (uint8_t)(r % 4U);

  switch (rotation) {
    case 0:
      madctl = MADCTL_MX | MADCTL_MY | MADCTL_RGB;
      break;
    case 1:
      madctl = MADCTL_MY | MADCTL_MV | MADCTL_RGB;
      break;
    case 2:
      madctl = MADCTL_RGB;
      break;
    case 3:
      madctl = MADCTL_MX | MADCTL_MV | MADCTL_RGB;
      break;
    default:
      madctl = MADCTL_MX | MADCTL_MY | MADCTL_RGB;
      break;
  }

  uint8_t st7735_cmds[] = {
    ST7735_MADCTL, 1, madctl, 0
  };

  sendCommands(st7735_cmds, 1U);
}
