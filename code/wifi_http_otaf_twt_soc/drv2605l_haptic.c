/***************************************************************************/ /**
 * @file drv2605l_haptic.c
 * @brief DRV2605L haptic motor controller task.
 ******************************************************************************/
#include "drv2605l_haptic.h"

#include "cmsis_os2.h"
#include "i2c_leader_example.h"
#include "sl_driver_gpio.h"
#include "sl_i2c_instances.h"
#include "sl_si91x_driver_gpio.h"

#include <stdio.h>

#define DRV2605L_REG_STATUS       0x00U
#define DRV2605L_REG_MODE         0x01U
#define DRV2605L_REG_RTP_INPUT    0x02U
#define DRV2605L_REG_LIBRARY      0x03U
#define DRV2605L_REG_WAVESEQ1     0x04U
#define DRV2605L_REG_WAVESEQ2     0x05U
#define DRV2605L_REG_GO           0x0CU
#define DRV2605L_REG_OVERDRIVE    0x0DU
#define DRV2605L_REG_SUSTAIN_POS  0x0EU
#define DRV2605L_REG_SUSTAIN_NEG  0x0FU
#define DRV2605L_REG_BRAKE        0x10U
#define DRV2605L_REG_AUDIO_CTRL   0x11U
#define DRV2605L_REG_FEEDBACK     0x1AU
#define DRV2605L_REG_CONTROL1     0x1BU
#define DRV2605L_REG_CONTROL2     0x1CU
#define DRV2605L_REG_CONTROL3     0x1DU

#define DRV2605L_MODE_INT_TRIG    0x00U
#define DRV2605L_MODE_RTP         0x05U
#define DRV2605L_MODE_STANDBY     0x40U
#define DRV2605L_ERM_LIBRARY      0x01U

#define DRV2605L_QUEUE_DEPTH      8U
#define DRV2605L_I2C_TIMEOUT_MS   100U
#define DRV2605L_IMU_IDLE_POLL_MS 20U
#define DRV2605L_IMU_READY_POLL_MS 20U
#define DRV2605L_IMU_PAUSE_SETTLE_MS 3U
#define DRV2605L_STARTUP_TEST_DELAY_MS 1000U

typedef enum {
  DRV2605L_CMD_PLAY_EFFECT = 0,
  DRV2605L_CMD_STOP,
  DRV2605L_CMD_RTP
} drv2605l_cmd_type_t;

typedef struct {
  drv2605l_cmd_type_t type;
  uint8_t value;
  uint16_t duration_ms;
  osSemaphoreId_t done_sem;
} drv2605l_cmd_t;

static osThreadId_t g_drv2605l_thread = NULL;
static osMessageQueueId_t g_drv2605l_queue = NULL;
static volatile uint8_t g_drv2605l_ready = 0U;
static const sl_gpio_t g_drv2605l_enable_gpio = {
  DRV2605L_ENABLE_PORT,
  DRV2605L_ENABLE_PIN,
};

static sl_i2c_status_t drv2605l_read_reg(uint8_t reg, uint8_t *value);

static void drv2605l_enable_pin_init(void)
{
  sl_status_t gpio_status;
  uint8_t pin_value = 0U;
  sl_si91x_gpio_pin_config_t pin_config = {
    { DRV2605L_ENABLE_PORT, DRV2605L_ENABLE_PIN },
    GPIO_OUTPUT
  };

  gpio_status = sl_gpio_set_configuration(pin_config);
  printf("DRV2605L enable GPIO config: port=%u pin=%u status=0x%lx\r\n",
         (unsigned int)DRV2605L_ENABLE_PORT,
         (unsigned int)DRV2605L_ENABLE_PIN,
         (unsigned long)gpio_status);

  gpio_status = sl_gpio_driver_set_pin((sl_gpio_t *)&g_drv2605l_enable_gpio);
  (void)sl_gpio_driver_get_pin((sl_gpio_t *)&g_drv2605l_enable_gpio, &pin_value);
  printf("DRV2605L enable GPIO set high: status=0x%lx read=%u\r\n",
         (unsigned long)gpio_status,
         (unsigned int)pin_value);
}

static sl_i2c_status_t drv2605l_i2c_driver_init(void)
{
#if DRV2605L_HAPTIC_INIT_I2C_DRIVER
  sl_i2c_status_t status;
  sl_i2c_config_t cfg;

  if (i2c_shared_bus_lock(DRV2605L_HAPTIC_I2C_INSTANCE, DRV2605L_I2C_TIMEOUT_MS) == 0U) {
    printf("DRV2605L I2C driver init lock timeout\r\n");
    return SL_I2C_TIMEOUT;
  }

  sl_i2c_init_instances();

  if (DRV2605L_HAPTIC_I2C_INSTANCE == SL_I2C0) {
    cfg = sl_i2c_i2c0_config;
  } else if (DRV2605L_HAPTIC_I2C_INSTANCE == SL_I2C1) {
    cfg = sl_i2c_i2c1_config;
  } else {
    i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
    return SL_I2C_INVALID_PARAMETER;
  }

  cfg.transfer_type = SL_I2C_USING_NON_DMA;
  cfg.i2c_callback = NULL;

  status = sl_i2c_driver_init(DRV2605L_HAPTIC_I2C_INSTANCE, &cfg);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L I2C driver init failed: status=%u\r\n", (unsigned int)status);
    i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
    return status;
  }

  status = sl_i2c_driver_configure_fifo_threshold(DRV2605L_HAPTIC_I2C_INSTANCE, 0U, 0U);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L I2C FIFO config failed: status=%u\r\n", (unsigned int)status);
    i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
    return status;
  }

  status = sl_i2c_driver_enable_repeated_start(DRV2605L_HAPTIC_I2C_INSTANCE, false);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L I2C repeated-start config failed: status=%u\r\n", (unsigned int)status);
    i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
    return status;
  }

  i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
  printf("DRV2605L I2C driver ready, inst=%d\r\n", (int)DRV2605L_HAPTIC_I2C_INSTANCE);
  return SL_I2C_SUCCESS;
#else
  return SL_I2C_SUCCESS;
#endif
}

static void drv2605l_debug_read(const char *name, uint8_t reg)
{
#if DRV2605L_HAPTIC_DEBUG
  uint8_t value = 0U;
  sl_i2c_status_t status = drv2605l_read_reg(reg, &value);

  printf("DRV2605L read %s(0x%02X): status=%u value=0x%02X\r\n",
         name,
         reg,
         (unsigned int)status,
         value);
#else
  (void)name;
  (void)reg;
#endif
}

static void drv2605l_wait_until_imu_not_recording(void)
{
  while (lsm6dso_get_storage_enabled() != 0U) {
    osDelay(DRV2605L_IMU_IDLE_POLL_MS);
  }
}

static void drv2605l_wait_until_i2c_ready(void)
{
  while (lsm6dso_pair_task_is_ready() == 0U) {
    osDelay(DRV2605L_IMU_READY_POLL_MS);
  }
}

static void drv2605l_begin_i2c_use(void)
{
  drv2605l_wait_until_imu_not_recording();
  lsm6dso_set_pair_sampling_paused(1U);
  osDelay(DRV2605L_IMU_PAUSE_SETTLE_MS);
  drv2605l_enable_pin_init();
  osDelay(20U);
}

static void drv2605l_end_i2c_use(void)
{
  lsm6dso_set_pair_sampling_paused(0U);
}

static sl_i2c_status_t drv2605l_write_reg(uint8_t reg, uint8_t value)
{
  sl_i2c_status_t status;
  uint8_t tx[2];

  tx[0] = reg;
  tx[1] = value;

  if (i2c_shared_bus_lock(DRV2605L_HAPTIC_I2C_INSTANCE, DRV2605L_I2C_TIMEOUT_MS) == 0U) {
    return SL_I2C_TIMEOUT;
  }

  status = sl_i2c_driver_enable_repeated_start(DRV2605L_HAPTIC_I2C_INSTANCE, false);
  if (status == SL_I2C_SUCCESS) {
    status = sl_i2c_driver_send_data_blocking(DRV2605L_HAPTIC_I2C_INSTANCE,
                                              DRV2605L_HAPTIC_I2C_ADDR,
                                              tx,
                                              sizeof(tx));
  }

  i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
  return status;
}

static sl_i2c_status_t drv2605l_read_reg(uint8_t reg, uint8_t *value)
{
  sl_i2c_status_t status;

  if (value == NULL) {
    return SL_I2C_INVALID_PARAMETER;
  }

  if (i2c_shared_bus_lock(DRV2605L_HAPTIC_I2C_INSTANCE, DRV2605L_I2C_TIMEOUT_MS) == 0U) {
    return SL_I2C_TIMEOUT;
  }

  status = sl_i2c_driver_enable_repeated_start(DRV2605L_HAPTIC_I2C_INSTANCE, true);
  if (status == SL_I2C_SUCCESS) {
    status = sl_i2c_driver_send_data_blocking(DRV2605L_HAPTIC_I2C_INSTANCE,
                                              DRV2605L_HAPTIC_I2C_ADDR,
                                              &reg,
                                              1U);
  }

  if (status == SL_I2C_SUCCESS) {
    status = sl_i2c_driver_enable_repeated_start(DRV2605L_HAPTIC_I2C_INSTANCE, false);
  }

  if (status == SL_I2C_SUCCESS) {
    status = sl_i2c_driver_receive_data_blocking(DRV2605L_HAPTIC_I2C_INSTANCE,
                                                 DRV2605L_HAPTIC_I2C_ADDR,
                                                 value,
                                                 1U);
  }

  i2c_shared_bus_unlock(DRV2605L_HAPTIC_I2C_INSTANCE);
  return status;
}

static sl_i2c_status_t drv2605l_init(void)
{
  sl_i2c_status_t status;
  uint8_t status_reg = 0U;

  printf("DRV2605L init start, inst=%d addr=0x%02X\r\n",
         (int)DRV2605L_HAPTIC_I2C_INSTANCE,
         DRV2605L_HAPTIC_I2C_ADDR);

  status = drv2605l_read_reg(DRV2605L_REG_STATUS, &status_reg);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L status read failed: status=%u\r\n", (unsigned int)status);
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_MODE, DRV2605L_MODE_INT_TRIG);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_RTP_INPUT, 0U);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_LIBRARY, DRV2605L_ERM_LIBRARY);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_WAVESEQ1, 0U);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_WAVESEQ2, 0U);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  printf("DRV2605L ready, inst=%d addr=0x%02X status=0x%02X\r\n",
         (int)DRV2605L_HAPTIC_I2C_INSTANCE,
         DRV2605L_HAPTIC_I2C_ADDR,
         status_reg);
  drv2605l_debug_read("MODE", DRV2605L_REG_MODE);
  drv2605l_debug_read("LIB", DRV2605L_REG_LIBRARY);
  drv2605l_debug_read("WAVE1", DRV2605L_REG_WAVESEQ1);
  drv2605l_debug_read("WAVE2", DRV2605L_REG_WAVESEQ2);
  drv2605l_debug_read("GO", DRV2605L_REG_GO);
  return SL_I2C_SUCCESS;
}

static sl_i2c_status_t drv2605l_play_effect_now(uint8_t effect_id)
{
  sl_i2c_status_t status;

  printf("DRV2605L play effect start: effect=%u\r\n", (unsigned int)effect_id);

  status = drv2605l_write_reg(DRV2605L_REG_MODE, DRV2605L_MODE_INT_TRIG);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L play mode write failed: status=%u\r\n", (unsigned int)status);
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_WAVESEQ1, effect_id);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L play wave1 write failed: status=%u\r\n", (unsigned int)status);
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_WAVESEQ2, 0U);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L play wave2 write failed: status=%u\r\n", (unsigned int)status);
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_GO, 1U);
  printf("DRV2605L play GO write status=%u\r\n", (unsigned int)status);
  drv2605l_debug_read("MODE", DRV2605L_REG_MODE);
  drv2605l_debug_read("WAVE1", DRV2605L_REG_WAVESEQ1);
  drv2605l_debug_read("GO", DRV2605L_REG_GO);

  return status;
}

static sl_i2c_status_t drv2605l_realtime_now(uint8_t magnitude, uint16_t duration_ms)
{
  sl_i2c_status_t status;

  printf("DRV2605L RTP start: magnitude=%u duration=%ums\r\n",
         (unsigned int)magnitude,
         (unsigned int)duration_ms);

  status = drv2605l_write_reg(DRV2605L_REG_MODE, DRV2605L_MODE_RTP);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L RTP mode write failed: status=%u\r\n", (unsigned int)status);
    return status;
  }

  status = drv2605l_write_reg(DRV2605L_REG_RTP_INPUT, magnitude);
  if (status != SL_I2C_SUCCESS) {
    printf("DRV2605L RTP input write failed: status=%u\r\n", (unsigned int)status);
    return status;
  }

  drv2605l_debug_read("MODE", DRV2605L_REG_MODE);
  drv2605l_debug_read("RTP", DRV2605L_REG_RTP_INPUT);

  osDelay(duration_ms);

  (void)drv2605l_write_reg(DRV2605L_REG_RTP_INPUT, 0U);
  (void)drv2605l_write_reg(DRV2605L_REG_MODE, DRV2605L_MODE_INT_TRIG);

  drv2605l_debug_read("RTP", DRV2605L_REG_RTP_INPUT);
  drv2605l_debug_read("MODE", DRV2605L_REG_MODE);

  return SL_I2C_SUCCESS;
}

static void drv2605l_haptic_task(void *arg)
{
  drv2605l_cmd_t cmd;

  (void)arg;

  osDelay(500U);
  drv2605l_enable_pin_init();
  osDelay(20U);

  drv2605l_wait_until_i2c_ready();
  drv2605l_begin_i2c_use();

  if (drv2605l_i2c_driver_init() != SL_I2C_SUCCESS) {
    printf("DRV2605L standalone I2C init failed\r\n");
  }

  if (drv2605l_init() == SL_I2C_SUCCESS) {
    g_drv2605l_ready = 1U;
    if (DRV2605L_HAPTIC_STARTUP_TEST != 0U) {
      osDelay(DRV2605L_STARTUP_TEST_DELAY_MS);
      printf("DRV2605L startup RTP test\r\n");
      (void)drv2605l_realtime_now(DRV2605L_HAPTIC_TEST_RTP_MAGNITUDE,
                                  DRV2605L_HAPTIC_TEST_RTP_DURATION_MS);
    }
  } else {
    printf("DRV2605L init failed, inst=%d addr=0x%02X\r\n",
           (int)DRV2605L_HAPTIC_I2C_INSTANCE,
           DRV2605L_HAPTIC_I2C_ADDR);
  }
  drv2605l_end_i2c_use();

  while (1) {
    if (osMessageQueueGet(g_drv2605l_queue, &cmd, NULL, osWaitForever) != osOK) {
      continue;
    }

    printf("DRV2605L cmd received: type=%u value=%u duration=%u\r\n",
           (unsigned int)cmd.type,
           (unsigned int)cmd.value,
           (unsigned int)cmd.duration_ms);
    drv2605l_begin_i2c_use();

    if (g_drv2605l_ready == 0U) {
      if (drv2605l_init() == SL_I2C_SUCCESS) {
        g_drv2605l_ready = 1U;
      } else {
        if (cmd.done_sem != NULL) {
          (void)osSemaphoreRelease(cmd.done_sem);
        }
        drv2605l_end_i2c_use();
        continue;
      }
    }

    if (cmd.type == DRV2605L_CMD_PLAY_EFFECT) {
      (void)drv2605l_play_effect_now(cmd.value);
    } else if (cmd.type == DRV2605L_CMD_STOP) {
      (void)drv2605l_write_reg(DRV2605L_REG_GO, 0U);
      (void)drv2605l_write_reg(DRV2605L_REG_RTP_INPUT, 0U);
    } else if (cmd.type == DRV2605L_CMD_RTP) {
      (void)drv2605l_realtime_now(cmd.value, cmd.duration_ms);
    }
    printf("DRV2605L cmd done: type=%u\r\n", (unsigned int)cmd.type);
    if (cmd.done_sem != NULL) {
      (void)osSemaphoreRelease(cmd.done_sem);
    }
    drv2605l_end_i2c_use();
  }
}

void drv2605l_haptic_task_start(void)
{
  static const osThreadAttr_t haptic_attr = {
    .name = "HapticTask",
    .stack_size = 1536U,
    .priority = osPriorityBelowNormal,
  };

  if (g_drv2605l_thread != NULL) {
    return;
  }

  if (g_drv2605l_queue == NULL) {
    g_drv2605l_queue = osMessageQueueNew(DRV2605L_QUEUE_DEPTH,
                                         sizeof(drv2605l_cmd_t),
                                         NULL);
  }

  if (g_drv2605l_queue == NULL) {
    printf("DRV2605L queue create failed\r\n");
    return;
  }

  g_drv2605l_thread = osThreadNew(drv2605l_haptic_task, NULL, &haptic_attr);
  if (g_drv2605l_thread == NULL) {
    printf("DRV2605L task create failed\r\n");
  }
}

uint8_t drv2605l_haptic_play_effect(uint8_t effect_id)
{
  drv2605l_cmd_t cmd = {
    .type = DRV2605L_CMD_PLAY_EFFECT,
    .value = effect_id,
    .duration_ms = 0U,
    .done_sem = NULL,
  };

  if (g_drv2605l_queue == NULL) {
    return 0U;
  }

  return (osMessageQueuePut(g_drv2605l_queue, &cmd, 0U, 0U) == osOK) ? 1U : 0U;
}

uint8_t drv2605l_haptic_stop(void)
{
  drv2605l_cmd_t cmd = {
    .type = DRV2605L_CMD_STOP,
    .value = 0U,
    .duration_ms = 0U,
    .done_sem = NULL,
  };

  if (g_drv2605l_queue == NULL) {
    return 0U;
  }

  return (osMessageQueuePut(g_drv2605l_queue, &cmd, 0U, 0U) == osOK) ? 1U : 0U;
}

uint8_t drv2605l_haptic_set_realtime(uint8_t magnitude, uint16_t duration_ms)
{
  drv2605l_cmd_t cmd = {
    .type = DRV2605L_CMD_RTP,
    .value = magnitude,
    .duration_ms = duration_ms,
    .done_sem = NULL,
  };
  osStatus_t status;

  if (g_drv2605l_queue == NULL) {
    printf("DRV2605L RTP queue not ready\r\n");
    return 0U;
  }

  status = osMessageQueuePut(g_drv2605l_queue, &cmd, 0U, 0U);
  if (status != osOK) {
    printf("DRV2605L RTP queue put failed: status=%d\r\n", (int)status);
    return 0U;
  }

  return 1U;
}

uint8_t drv2605l_haptic_set_realtime_wait(uint8_t magnitude,
                                          uint16_t duration_ms,
                                          uint32_t timeout_ms)
{
  drv2605l_cmd_t cmd = {
    .type = DRV2605L_CMD_RTP,
    .value = magnitude,
    .duration_ms = duration_ms,
    .done_sem = NULL,
  };
  osSemaphoreId_t done_sem;
  osStatus_t status;

  if (g_drv2605l_queue == NULL) {
    printf("DRV2605L RTP wait queue not ready\r\n");
    return 0U;
  }

  done_sem = osSemaphoreNew(1U, 0U, NULL);
  if (done_sem == NULL) {
    printf("DRV2605L RTP wait semaphore create failed\r\n");
    return 0U;
  }

  cmd.done_sem = done_sem;
  status = osMessageQueuePut(g_drv2605l_queue, &cmd, 0U, 0U);
  if (status != osOK) {
    printf("DRV2605L RTP wait queue put failed: status=%d\r\n", (int)status);
    (void)osSemaphoreDelete(done_sem);
    return 0U;
  }

  status = osSemaphoreAcquire(done_sem, timeout_ms);
  (void)osSemaphoreDelete(done_sem);
  if (status != osOK) {
    printf("DRV2605L RTP wait timeout/status=%d\r\n", (int)status);
    return 0U;
  }

  return 1U;
}
