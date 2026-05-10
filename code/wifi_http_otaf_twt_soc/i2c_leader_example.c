/***************************************************************************/ /**
* @file i2c_leader_example.c
* @brief Dual LSM6DSO IMU readers over two I2C instances
*******************************************************************************
* # License
* <b>Copyright 2023 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/
#include "i2c_leader_example.h"

#include "cmsis_os2.h"
#include "rsi_debug.h"
#include "sl_i2c_instances.h"
#include "sl_si91x_i2c.h"

#include <stdio.h>
#include <string.h>

extern volatile uint8_t imu_print_enabled;

#ifndef INSTANCE_ZERO
#define INSTANCE_ZERO 0
#endif

#ifndef INSTANCE_ONE
#define INSTANCE_ONE  1
#endif

#define LSM6DSO_IMU0_ADDR        0x6B
#define LSM6DSO_IMU1_ADDR        0x6A

/* Debug isolation step 3:
 * run both IMUs. DRV2605L shares I2C0 with IMU0, so its driver pauses IMU
 * sampling and uses the shared I2C0 bus lock before haptic transactions.
 */
#define LSM6DSO_ENABLE_IMU0      1U
#define LSM6DSO_ENABLE_IMU1      1U
#define LSM6DSO_READ_IMU0        1U
#define LSM6DSO_READ_IMU1        1U

#define LSM6DSO_WHO_AM_I_REG     0x0F
#define LSM6DSO_CTRL1_XL_REG     0x10
#define LSM6DSO_CTRL2_G_REG      0x11
#define LSM6DSO_CTRL3_C_REG      0x12
#define LSM6DSO_OUTX_L_G_REG     0x22
#define LSM6DSO_WHO_AM_I_VALUE   0x6C

#define LSM6DSO_CTRL3_C_VALUE    0x44U  /* BDU=1, IF_INC=1 */
#define LSM6DSO_CTRL1_XL_VALUE   0x7CU  /* accel ODR=833Hz, FS=8g */
#define LSM6DSO_CTRL2_G_VALUE    0x48U  /* gyro ODR=104Hz, FS=1000dps */

#define LSM6DSO_IMU_BURST_LEN    12U
#define I2C_TX_FIFO_THRESHOLD    0U
#define I2C_RX_FIFO_THRESHOLD    0U
#define I2C_SAMPLE_PERIOD_MS     2U
#define I2C_PRINT_PERIOD_MS      50U
#define I2C_DMA_READ_TIMEOUT_MS  20U
#define LSM6DSO_USE_DMA_READ     0U
#define LSM6DSO_DMA_SMOKE_TEST_ENABLED 0U
#define LSM6DSO_DMA_SMOKE_TEST_IMU_INDEX 0U

#define LSM6DSO_GYRO_SENS_DPS_LSB 0.035f
#define LSM6DSO_ACC_SENS_MG_LSB   0.244f
#define LSM6DSO_MG_TO_MPS2        0.00980665f

/* Avoid depending on a project-specific SL_I2C_ERROR enum value. */
#define LSM6DSO_GENERIC_FAIL      ((sl_i2c_status_t)1)

typedef struct {
  uint16_t sequence;
  uint16_t tick_delta_ms;
  int8_t gx_i8;
  int8_t gy_i8;
  int8_t gz_i8;
  int8_t ax_i8;
  int8_t ay_i8;
  int8_t az_i8;
} lsm6dso_stored_sample_t;

typedef struct {
  uint8_t index;
  sl_i2c_instance_t inst;
  uint8_t addr;
  const char *name;
  osThreadId_t thread_id;
  osMutexId_t lock;
  osSemaphoreId_t dma_sem;
  volatile uint32_t dma_status;
  lsm6dso_stored_sample_t ring[LSM6DSO_IMU_STORAGE_CAPACITY];
  uint16_t ring_head;
  uint16_t ring_count;
  uint8_t latest_valid;
  uint32_t total_samples;
  uint32_t overwritten_samples;
  uint32_t storage_base_tick;
  uint32_t last_print_tick;
} imu_task_ctx_t;

static imu_task_ctx_t g_imu_ctx[] = {
  {
    .index = 0U,
    .inst = INSTANCE_ZERO,
    .addr = LSM6DSO_IMU0_ADDR,
    .name = "IMU0",
  },
  {
    .index = 1U,
    .inst = INSTANCE_ONE,
    .addr = LSM6DSO_IMU1_ADDR,
    .name = "IMU1",
  },
};

static uint8_t g_dual_tasks_started = 0U;
static volatile uint8_t g_storage_enabled = 0U;
static volatile uint8_t g_pair_sampling_paused = 0U;
static volatile uint8_t g_pair_task_ready = 0U;
static osMutexId_t g_i2c_init_lock = NULL;
static osMutexId_t g_i2c_dma_read_lock = NULL;
static osMutexId_t g_i2c_bus_locks[2] = { NULL, NULL };

static void lsm6dso_i2c_dma_callback(sl_i2c_instance_t i2c_instance,
                                     uint32_t status);

uint8_t i2c_shared_bus_lock(sl_i2c_instance_t inst, uint32_t timeout_ms)
{
  osMutexId_t lock;
  uint32_t timeout_ticks;

  if (inst >= (sizeof(g_i2c_bus_locks) / sizeof(g_i2c_bus_locks[0]))) {
    return 0U;
  }

  lock = g_i2c_bus_locks[inst];
  if (lock == NULL) {
    return 1U;
  }

  timeout_ticks = (timeout_ms == 0U) ? 0U : timeout_ms;
  return (osMutexAcquire(lock, timeout_ticks) == osOK) ? 1U : 0U;
}

void i2c_shared_bus_unlock(sl_i2c_instance_t inst)
{
  if (inst >= (sizeof(g_i2c_bus_locks) / sizeof(g_i2c_bus_locks[0]))) {
    return;
  }

  if (g_i2c_bus_locks[inst] != NULL) {
    (void)osMutexRelease(g_i2c_bus_locks[inst]);
  }
}

static sl_i2c_status_t lsm6dso_select_config(sl_i2c_instance_t inst,
                                             sl_i2c_config_t *cfg,
                                             uint8_t use_dma)
{
  if (cfg == NULL) {
    return LSM6DSO_GENERIC_FAIL;
  }

  if (inst == INSTANCE_ZERO) {
    *cfg = sl_i2c_i2c0_config;
  } else if (inst == INSTANCE_ONE) {
    *cfg = sl_i2c_i2c1_config;
  } else {
    return LSM6DSO_GENERIC_FAIL;
  }

  if (use_dma != 0U) {
    cfg->transfer_type = SL_I2C_USING_DMA;
    cfg->i2c_callback = lsm6dso_i2c_dma_callback;
  } else {
    (void)use_dma;
    cfg->i2c_callback = NULL;
  }

  return SL_I2C_SUCCESS;
}

static imu_task_ctx_t *lsm6dso_ctx_for_inst(sl_i2c_instance_t inst)
{
  for (uint32_t i = 0U; i < (sizeof(g_imu_ctx) / sizeof(g_imu_ctx[0])); i++) {
    if (g_imu_ctx[i].inst == inst) {
      return &g_imu_ctx[i];
    }
  }

  return NULL;
}

#if (LSM6DSO_USE_DMA_READ || LSM6DSO_DMA_SMOKE_TEST_ENABLED)
static sl_i2c_status_t lsm6dso_get_dma_config(sl_i2c_instance_t inst,
                                              sl_i2c_dma_config_t *cfg)
{
  if (cfg == NULL) {
    return LSM6DSO_GENERIC_FAIL;
  }

  if (inst == SL_I2C0) {
    cfg->dma_tx_channel = SL_I2C0_DMA_TX_CHANNEL;
    cfg->dma_rx_channel = SL_I2C0_DMA_RX_CHANNEL;
  } else if (inst == SL_I2C1) {
    cfg->dma_tx_channel = SL_I2C1_DMA_TX_CHANNEL;
    cfg->dma_rx_channel = SL_I2C1_DMA_RX_CHANNEL;
  } else {
    return LSM6DSO_GENERIC_FAIL;
  }

  return SL_I2C_SUCCESS;
}
#endif

static void lsm6dso_i2c_dma_callback(sl_i2c_instance_t i2c_instance,
                                     uint32_t status)
{
  imu_task_ctx_t *ctx = lsm6dso_ctx_for_inst(i2c_instance);

  if (ctx == NULL) {
    return;
  }

  ctx->dma_status = status;
  if (ctx->dma_sem != NULL) {
    (void)osSemaphoreRelease(ctx->dma_sem);
  }
}

static sl_i2c_status_t lsm6dso_write_reg_inst(sl_i2c_instance_t inst,
                                              uint8_t addr,
                                              uint8_t reg,
                                              uint8_t value)
{
  sl_i2c_status_t status;
  uint8_t tx[2];

  tx[0] = reg;
  tx[1] = value;

  if (i2c_shared_bus_lock(inst, osWaitForever) == 0U) {
    return LSM6DSO_GENERIC_FAIL;
  }

  status = sl_i2c_driver_enable_repeated_start(inst, false);
  if (status != SL_I2C_SUCCESS) {
    i2c_shared_bus_unlock(inst);
    return status;
  }

  status = sl_i2c_driver_send_data_blocking(inst, addr, tx, sizeof(tx));
  i2c_shared_bus_unlock(inst);

  return status;
}

static sl_i2c_status_t lsm6dso_read_reg_inst(sl_i2c_instance_t inst,
                                             uint8_t addr,
                                             uint8_t reg,
                                             uint8_t *data,
                                             uint16_t len)
{
  sl_i2c_status_t status;

  if ((data == NULL) || (len == 0U)) {
    return LSM6DSO_GENERIC_FAIL;
  }

  if (i2c_shared_bus_lock(inst, osWaitForever) == 0U) {
    return LSM6DSO_GENERIC_FAIL;
  }

  status = sl_i2c_driver_enable_repeated_start(inst, true);
  if (status != SL_I2C_SUCCESS) {
    i2c_shared_bus_unlock(inst);
    return status;
  }

  status = sl_i2c_driver_send_data_blocking(inst, addr, &reg, 1U);
  if (status != SL_I2C_SUCCESS) {
    (void)sl_i2c_driver_enable_repeated_start(inst, false);
    i2c_shared_bus_unlock(inst);
    return status;
  }

  status = sl_i2c_driver_enable_repeated_start(inst, false);
  if (status != SL_I2C_SUCCESS) {
    i2c_shared_bus_unlock(inst);
    return status;
  }

  status = sl_i2c_driver_receive_data_blocking(inst, addr, data, len);
  i2c_shared_bus_unlock(inst);

  return status;
}

#if (LSM6DSO_USE_DMA_READ || LSM6DSO_DMA_SMOKE_TEST_ENABLED)
static sl_i2c_status_t lsm6dso_read_reg_dma_inst(sl_i2c_instance_t inst,
                                                 uint8_t addr,
                                                 uint8_t reg,
                                                 uint8_t *data,
                                                 uint16_t len)
{
  imu_task_ctx_t *ctx;
  sl_i2c_dma_config_t dma_cfg;
  sl_i2c_status_t status;
  osStatus_t os_status;

  if ((data == NULL) || (len == 0U)) {
    return LSM6DSO_GENERIC_FAIL;
  }

  ctx = lsm6dso_ctx_for_inst(inst);
  if ((ctx == NULL) || (ctx->dma_sem == NULL)) {
    return LSM6DSO_GENERIC_FAIL;
  }

  status = lsm6dso_get_dma_config(inst, &dma_cfg);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  if (g_i2c_dma_read_lock != NULL) {
    (void)osMutexAcquire(g_i2c_dma_read_lock, osWaitForever);
  }

  while (osSemaphoreAcquire(ctx->dma_sem, 0U) == osOK) {
  }
  ctx->dma_status = LSM6DSO_GENERIC_FAIL;

  status = sl_i2c_driver_enable_repeated_start(inst, true);
  if (status != SL_I2C_SUCCESS) {
    goto read_done;
  }

  status = sl_i2c_driver_send_data_blocking(inst, addr, &reg, 1U);
  if (status != SL_I2C_SUCCESS) {
    (void)sl_i2c_driver_enable_repeated_start(inst, false);
    goto read_done;
  }

  status = sl_i2c_driver_enable_repeated_start(inst, false);
  if (status != SL_I2C_SUCCESS) {
    goto read_done;
  }

  status = sl_i2c_driver_receive_data_non_blocking(inst,
                                                   addr,
                                                   data,
                                                   len,
                                                   &dma_cfg);
  if (status != SL_I2C_SUCCESS) {
    goto read_done;
  }

  os_status = osSemaphoreAcquire(ctx->dma_sem, I2C_DMA_READ_TIMEOUT_MS);
  if (os_status != osOK) {
    status = SL_I2C_TIMEOUT;
    goto read_done;
  }

  if (ctx->dma_status == SL_I2C_DATA_TRANSFER_COMPLETE) {
    status = SL_I2C_SUCCESS;
  } else if (ctx->dma_status == SL_I2C_DMA_TRANSFER_ERROR) {
    status = SL_I2C_DMA_TRANSFER_ERROR;
  } else {
    status = LSM6DSO_GENERIC_FAIL;
  }

read_done:
  if (g_i2c_dma_read_lock != NULL) {
    (void)osMutexRelease(g_i2c_dma_read_lock);
  }

  return status;
}
#endif

sl_i2c_status_t lsm6dso_read_imu_data(sl_i2c_instance_t inst,
                                      uint8_t addr,
                                      lsm6dso_imu_data_t *out)
{
  sl_i2c_status_t status;
  uint8_t buf[LSM6DSO_IMU_BURST_LEN];

  if (out == NULL) {
    return LSM6DSO_GENERIC_FAIL;
  }

#if LSM6DSO_USE_DMA_READ
  status = lsm6dso_read_reg_dma_inst(inst, addr, LSM6DSO_OUTX_L_G_REG,
                                     buf, sizeof(buf));
#else
  status = lsm6dso_read_reg_inst(inst, addr, LSM6DSO_OUTX_L_G_REG,
                                 buf, sizeof(buf));
#endif
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  out->gx_raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  out->gy_raw = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
  out->gz_raw = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
  out->ax_raw = (int16_t)((uint16_t)buf[7] << 8 | buf[6]);
  out->ay_raw = (int16_t)((uint16_t)buf[9] << 8 | buf[8]);
  out->az_raw = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);

  out->gx_dps = ((float)out->gx_raw) * LSM6DSO_GYRO_SENS_DPS_LSB;
  out->gy_dps = ((float)out->gy_raw) * LSM6DSO_GYRO_SENS_DPS_LSB;
  out->gz_dps = ((float)out->gz_raw) * LSM6DSO_GYRO_SENS_DPS_LSB;

  out->ax_mps2 = ((float)out->ax_raw) * LSM6DSO_ACC_SENS_MG_LSB * LSM6DSO_MG_TO_MPS2;
  out->ay_mps2 = ((float)out->ay_raw) * LSM6DSO_ACC_SENS_MG_LSB * LSM6DSO_MG_TO_MPS2;
  out->az_mps2 = ((float)out->az_raw) * LSM6DSO_ACC_SENS_MG_LSB * LSM6DSO_MG_TO_MPS2;

  return SL_I2C_SUCCESS;
}

#if LSM6DSO_READ_IMU0 && LSM6DSO_READ_IMU1
static int8_t lsm6dso_raw_to_i8(int16_t raw)
{
  return (int8_t)(raw / 256);
}
#endif

static int16_t lsm6dso_i8_to_raw(int8_t value)
{
  return (int16_t)value * 256;
}

static void lsm6dso_convert_raw_sample(lsm6dso_imu_sample_t *out)
{
  out->data.gx_dps = ((float)out->data.gx_raw) * LSM6DSO_GYRO_SENS_DPS_LSB;
  out->data.gy_dps = ((float)out->data.gy_raw) * LSM6DSO_GYRO_SENS_DPS_LSB;
  out->data.gz_dps = ((float)out->data.gz_raw) * LSM6DSO_GYRO_SENS_DPS_LSB;

  out->data.ax_mps2 = ((float)out->data.ax_raw) * LSM6DSO_ACC_SENS_MG_LSB * LSM6DSO_MG_TO_MPS2;
  out->data.ay_mps2 = ((float)out->data.ay_raw) * LSM6DSO_ACC_SENS_MG_LSB * LSM6DSO_MG_TO_MPS2;
  out->data.az_mps2 = ((float)out->data.az_raw) * LSM6DSO_ACC_SENS_MG_LSB * LSM6DSO_MG_TO_MPS2;
}

static void lsm6dso_expand_stored_sample(const imu_task_ctx_t *ctx,
                                         const lsm6dso_stored_sample_t *stored,
                                         lsm6dso_imu_sample_t *out)
{
  memset(out, 0, sizeof(*out));

  out->sequence = stored->sequence;
  out->tick_ms = ctx->storage_base_tick + stored->tick_delta_ms;
  out->data.gx_raw = lsm6dso_i8_to_raw(stored->gx_i8);
  out->data.gy_raw = lsm6dso_i8_to_raw(stored->gy_i8);
  out->data.gz_raw = lsm6dso_i8_to_raw(stored->gz_i8);
  out->data.ax_raw = lsm6dso_i8_to_raw(stored->ax_i8);
  out->data.ay_raw = lsm6dso_i8_to_raw(stored->ay_i8);
  out->data.az_raw = lsm6dso_i8_to_raw(stored->az_i8);

  lsm6dso_convert_raw_sample(out);
}

static sl_i2c_status_t lsm6dso_init_for_ctx(imu_task_ctx_t *ctx)
{
  sl_i2c_status_t status;
  sl_i2c_config_t cfg;
  uint8_t who = 0U;

  if (ctx == NULL) {
    return LSM6DSO_GENERIC_FAIL;
  }

  if (g_i2c_init_lock != NULL) {
    (void)osMutexAcquire(g_i2c_init_lock, osWaitForever);
  }

  status = lsm6dso_select_config(ctx->inst, &cfg, 0U);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = sl_i2c_driver_init(ctx->inst, &cfg);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = sl_i2c_driver_configure_fifo_threshold(ctx->inst,
                                                  I2C_TX_FIFO_THRESHOLD,
                                                  I2C_RX_FIFO_THRESHOLD);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = sl_i2c_driver_enable_repeated_start(ctx->inst, false);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = lsm6dso_read_reg_inst(ctx->inst, ctx->addr,
                                 LSM6DSO_WHO_AM_I_REG, &who, 1U);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  if (who != LSM6DSO_WHO_AM_I_VALUE) {
    printf("%s WHO_AM_I mismatch: got 0x%02X expected 0x%02X\r\n",
           ctx->name, who, LSM6DSO_WHO_AM_I_VALUE);
    status = LSM6DSO_GENERIC_FAIL;
    goto init_done;
  }

  status = lsm6dso_write_reg_inst(ctx->inst, ctx->addr,
                                  LSM6DSO_CTRL3_C_REG, LSM6DSO_CTRL3_C_VALUE);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = lsm6dso_write_reg_inst(ctx->inst, ctx->addr,
                                  LSM6DSO_CTRL1_XL_REG, LSM6DSO_CTRL1_XL_VALUE);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = lsm6dso_write_reg_inst(ctx->inst, ctx->addr,
                                  LSM6DSO_CTRL2_G_REG, LSM6DSO_CTRL2_G_VALUE);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

#if LSM6DSO_USE_DMA_READ
  status = lsm6dso_select_config(ctx->inst, &cfg, 1U);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = sl_i2c_driver_init(ctx->inst, &cfg);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = sl_i2c_driver_configure_fifo_threshold(ctx->inst,
                                                  I2C_TX_FIFO_THRESHOLD,
                                                  I2C_RX_FIFO_THRESHOLD);
  if (status != SL_I2C_SUCCESS) {
    goto init_done;
  }

  status = sl_i2c_driver_enable_repeated_start(ctx->inst, false);
#endif

init_done:
  if (g_i2c_init_lock != NULL) {
    (void)osMutexRelease(g_i2c_init_lock);
  }

  return status;
}

#if LSM6DSO_DMA_SMOKE_TEST_ENABLED
static sl_i2c_status_t lsm6dso_configure_i2c_mode(imu_task_ctx_t *ctx,
                                                  uint8_t use_dma)
{
  sl_i2c_status_t status;
  sl_i2c_config_t cfg;

  if (ctx == NULL) {
    return LSM6DSO_GENERIC_FAIL;
  }

  status = lsm6dso_select_config(ctx->inst, &cfg, use_dma);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  status = sl_i2c_driver_init(ctx->inst, &cfg);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  status = sl_i2c_driver_configure_fifo_threshold(ctx->inst,
                                                  I2C_TX_FIFO_THRESHOLD,
                                                  I2C_RX_FIFO_THRESHOLD);
  if (status != SL_I2C_SUCCESS) {
    return status;
  }

  return sl_i2c_driver_enable_repeated_start(ctx->inst, false);
}

static void lsm6dso_dma_smoke_test(imu_task_ctx_t *ctx)
{
  sl_i2c_status_t status;
  sl_i2c_status_t restore_status;
  uint8_t buf[LSM6DSO_IMU_BURST_LEN];

  if ((ctx == NULL) || (ctx->index != LSM6DSO_DMA_SMOKE_TEST_IMU_INDEX)) {
    return;
  }

  printf("%s DMA smoke test start, inst=%d addr=0x%02X\r\n",
         ctx->name, (int)ctx->inst, ctx->addr);

  if (g_i2c_init_lock != NULL) {
    (void)osMutexAcquire(g_i2c_init_lock, osWaitForever);
  }

  (void)sl_i2c_driver_deinit(ctx->inst);
  status = lsm6dso_configure_i2c_mode(ctx, 1U);
  if (status == SL_I2C_SUCCESS) {
    memset(buf, 0, sizeof(buf));
    status = lsm6dso_read_reg_dma_inst(ctx->inst,
                                       ctx->addr,
                                       LSM6DSO_OUTX_L_G_REG,
                                       buf,
                                       sizeof(buf));
  }

  (void)sl_i2c_driver_deinit(ctx->inst);
  restore_status = lsm6dso_configure_i2c_mode(ctx, 0U);

  if (g_i2c_init_lock != NULL) {
    (void)osMutexRelease(g_i2c_init_lock);
  }

  if (status == SL_I2C_SUCCESS) {
    printf("%s DMA smoke test OK: %02X %02X %02X %02X\r\n",
           ctx->name, buf[0], buf[1], buf[2], buf[3]);
  } else {
    printf("%s DMA smoke test failed: status=%u\r\n",
           ctx->name, (unsigned int)status);
  }

  if (restore_status != SL_I2C_SUCCESS) {
    printf("%s DMA smoke test restore failed: status=%u\r\n",
           ctx->name, (unsigned int)restore_status);
  }
}
#endif

#if LSM6DSO_READ_IMU0 && LSM6DSO_READ_IMU1
static void lsm6dso_store_one_locked(imu_task_ctx_t *ctx,
                                     const lsm6dso_imu_sample_t *sample,
                                     uint32_t pair_sequence,
                                     uint32_t pair_tick)
{
  lsm6dso_stored_sample_t stored_sample;

  if ((ctx == NULL) || (sample == NULL)) {
    return;
  }

  memset(&stored_sample, 0, sizeof(stored_sample));
  stored_sample.gx_i8 = lsm6dso_raw_to_i8(sample->data.gx_raw);
  stored_sample.gy_i8 = lsm6dso_raw_to_i8(sample->data.gy_raw);
  stored_sample.gz_i8 = lsm6dso_raw_to_i8(sample->data.gz_raw);
  stored_sample.ax_i8 = lsm6dso_raw_to_i8(sample->data.ax_raw);
  stored_sample.ay_i8 = lsm6dso_raw_to_i8(sample->data.ay_raw);
  stored_sample.az_i8 = lsm6dso_raw_to_i8(sample->data.az_raw);

  if (ctx->total_samples == 0U) {
    ctx->storage_base_tick = pair_tick;
  }

  ctx->total_samples = pair_sequence;
  stored_sample.sequence = (uint16_t)pair_sequence;
  stored_sample.tick_delta_ms = (uint16_t)(pair_tick - ctx->storage_base_tick);

  if (ctx->ring_count >= LSM6DSO_IMU_STORAGE_CAPACITY) {
    ctx->overwritten_samples++;
  }

  ctx->ring[ctx->ring_head] = stored_sample;
  ctx->ring_head = (uint16_t)((ctx->ring_head + 1U) % LSM6DSO_IMU_STORAGE_CAPACITY);
  if (ctx->ring_count < LSM6DSO_IMU_STORAGE_CAPACITY) {
    ctx->ring_count++;
  }
  ctx->latest_valid = 1U;
}

static void lsm6dso_store_pair(const lsm6dso_imu_sample_t *imu0_sample,
                               const lsm6dso_imu_sample_t *imu1_sample,
                               uint32_t pair_tick)
{
  uint32_t pair_sequence;

  if ((imu0_sample == NULL) || (imu1_sample == NULL)) {
    return;
  }

  if (g_storage_enabled == 0U) {
    return;
  }

  if (g_imu_ctx[0].lock != NULL) {
    (void)osMutexAcquire(g_imu_ctx[0].lock, osWaitForever);
  }
  if (g_imu_ctx[1].lock != NULL) {
    (void)osMutexAcquire(g_imu_ctx[1].lock, osWaitForever);
  }

  pair_sequence = g_imu_ctx[0].total_samples + 1U;
  lsm6dso_store_one_locked(&g_imu_ctx[0], imu0_sample, pair_sequence, pair_tick);
  lsm6dso_store_one_locked(&g_imu_ctx[1], imu1_sample, pair_sequence, pair_tick);

  if (g_imu_ctx[1].lock != NULL) {
    (void)osMutexRelease(g_imu_ctx[1].lock);
  }
  if (g_imu_ctx[0].lock != NULL) {
    (void)osMutexRelease(g_imu_ctx[0].lock);
  }
}
#endif

static void imu_pair_task_entry(void *arg)
{
#if LSM6DSO_ENABLE_IMU0
  sl_i2c_status_t imu0_status = LSM6DSO_GENERIC_FAIL;
#endif
  sl_i2c_status_t imu1_status = LSM6DSO_GENERIC_FAIL;
  uint32_t last_error_print_tick = 0U;

  (void)arg;

#if LSM6DSO_ENABLE_IMU0
  imu0_status = lsm6dso_init_for_ctx(&g_imu_ctx[0]);
  if (imu0_status != SL_I2C_SUCCESS) {
    printf("%s init failed, inst=%d addr=0x%02X status=%u\r\n",
           g_imu_ctx[0].name,
           (int)g_imu_ctx[0].inst,
           g_imu_ctx[0].addr,
           (unsigned int)imu0_status);
    osThreadExit();
  }
#endif

#if LSM6DSO_ENABLE_IMU1
  imu1_status = lsm6dso_init_for_ctx(&g_imu_ctx[1]);
  if (imu1_status != SL_I2C_SUCCESS) {
    printf("%s init failed, inst=%d addr=0x%02X status=%u\r\n",
           g_imu_ctx[1].name,
           (int)g_imu_ctx[1].inst,
           g_imu_ctx[1].addr,
           (unsigned int)imu1_status);
    osThreadExit();
  }
#endif

#if LSM6DSO_ENABLE_IMU0
  printf("%s ready, inst=%d addr=0x%02X\r\n",
         g_imu_ctx[0].name, (int)g_imu_ctx[0].inst, g_imu_ctx[0].addr);
#endif
#if LSM6DSO_ENABLE_IMU1
  printf("%s ready, inst=%d addr=0x%02X\r\n",
         g_imu_ctx[1].name, (int)g_imu_ctx[1].inst, g_imu_ctx[1].addr);
#endif
#if LSM6DSO_READ_IMU0 && LSM6DSO_READ_IMU1
  printf("IMU pair task ready: read IMU0 then IMU1 per stored pair\r\n");
#else
  printf("IMU debug isolation: enable0=%u read0=%u enable1=%u read1=%u\r\n",
         (unsigned int)LSM6DSO_ENABLE_IMU0,
         (unsigned int)LSM6DSO_READ_IMU0,
         (unsigned int)LSM6DSO_ENABLE_IMU1,
         (unsigned int)LSM6DSO_READ_IMU1);
#endif
  g_pair_task_ready = 1U;

  while (1) {
    lsm6dso_imu_sample_t imu0_sample;
    lsm6dso_imu_sample_t imu1_sample;
    uint32_t pair_tick;

    if (g_pair_sampling_paused != 0U) {
      osDelay(I2C_SAMPLE_PERIOD_MS);
      continue;
    }

    memset(&imu0_sample, 0, sizeof(imu0_sample));
    memset(&imu1_sample, 0, sizeof(imu1_sample));

#if LSM6DSO_READ_IMU0
    imu0_status = lsm6dso_read_imu_data(g_imu_ctx[0].inst,
                                        g_imu_ctx[0].addr,
                                        &imu0_sample.data);
#endif
#if LSM6DSO_READ_IMU1
    imu1_status = lsm6dso_read_imu_data(g_imu_ctx[1].inst,
                                        g_imu_ctx[1].addr,
                                        &imu1_sample.data);
#endif
    pair_tick = osKernelGetTickCount();
    imu0_sample.tick_ms = pair_tick;
    imu1_sample.tick_ms = pair_tick;

#if LSM6DSO_READ_IMU0 && LSM6DSO_READ_IMU1
    if ((imu0_status == SL_I2C_SUCCESS) && (imu1_status == SL_I2C_SUCCESS)) {
      lsm6dso_store_pair(&imu0_sample, &imu1_sample, pair_tick);

      if ((imu_print_enabled != 0U) &&
          ((pair_tick - g_imu_ctx[0].last_print_tick) >= I2C_PRINT_PERIOD_MS)) {
        g_imu_ctx[0].last_print_tick = pair_tick;
        g_imu_ctx[1].last_print_tick = pair_tick;
        printf("IMU pair GYR dps: IMU0[%.3f %.3f %.3f] IMU1[%.3f %.3f %.3f] | ACC m/s^2: IMU0[%.3f %.3f %.3f] IMU1[%.3f %.3f %.3f]\r\n",
               imu0_sample.data.gx_dps, imu0_sample.data.gy_dps, imu0_sample.data.gz_dps,
               imu1_sample.data.gx_dps, imu1_sample.data.gy_dps, imu1_sample.data.gz_dps,
               imu0_sample.data.ax_mps2, imu0_sample.data.ay_mps2, imu0_sample.data.az_mps2,
               imu1_sample.data.ax_mps2, imu1_sample.data.ay_mps2, imu1_sample.data.az_mps2);
      }
    } else {
      uint32_t now = osKernelGetTickCount();
      if ((now - last_error_print_tick) >= 1000U) {
        last_error_print_tick = now;
        printf("IMU pair read failed, imu0=%u imu1=%u\r\n",
               (unsigned int)imu0_status,
               (unsigned int)imu1_status);
      }
    }
#elif LSM6DSO_READ_IMU1
    if (imu1_status == SL_I2C_SUCCESS) {
      if ((imu_print_enabled != 0U) &&
          ((pair_tick - g_imu_ctx[1].last_print_tick) >= I2C_PRINT_PERIOD_MS)) {
        g_imu_ctx[1].last_print_tick = pair_tick;
        printf("IMU1 GYR dps: [%.3f %.3f %.3f] | ACC m/s^2: [%.3f %.3f %.3f]\r\n",
               imu1_sample.data.gx_dps, imu1_sample.data.gy_dps, imu1_sample.data.gz_dps,
               imu1_sample.data.ax_mps2, imu1_sample.data.ay_mps2, imu1_sample.data.az_mps2);
      }
    } else {
      uint32_t now = osKernelGetTickCount();
      if ((now - last_error_print_tick) >= 1000U) {
        last_error_print_tick = now;
        printf("IMU1 read failed, status=%u\r\n", (unsigned int)imu1_status);
      }
    }
#else
    osThreadExit();
#endif

    osDelay(I2C_SAMPLE_PERIOD_MS);
  }
}

void i2c_dual_imu_tasks_start(void)
{
  static const osThreadAttr_t imu_pair_attr = {
    .name = "IMUPairTask",
    .stack_size = 4096U,
    .priority = osPriorityNormal,
  };
  static const osMutexAttr_t imu0_mutex_attr = {
    .name = "IMU0Lock",
  };
  static const osMutexAttr_t imu1_mutex_attr = {
    .name = "IMU1Lock",
  };
  static const osMutexAttr_t i2c_init_mutex_attr = {
    .name = "I2CInitLock",
  };
  static const osMutexAttr_t i2c_dma_read_mutex_attr = {
    .name = "I2CDMAReadLock",
  };
  static const osMutexAttr_t i2c0_bus_mutex_attr = {
    .name = "I2C0BusLock",
  };
  static const osMutexAttr_t i2c1_bus_mutex_attr = {
    .name = "I2C1BusLock",
  };
  static const osSemaphoreAttr_t imu0_dma_sem_attr = {
    .name = "IMU0DMASem",
  };
  static const osSemaphoreAttr_t imu1_dma_sem_attr = {
    .name = "IMU1DMASem",
  };

  if (g_dual_tasks_started != 0U) {
    return;
  }

  DEBUGINIT();

#if LSM6DSO_ENABLE_IMU0
  sl_i2c_init_instances();
#else
  printf("IMU: skip global I2C instance init while IMU0/I2C0 is disabled\r\n");
#endif

  g_i2c_init_lock = osMutexNew(&i2c_init_mutex_attr);
  g_i2c_dma_read_lock = osMutexNew(&i2c_dma_read_mutex_attr);
  g_i2c_bus_locks[INSTANCE_ZERO] = osMutexNew(&i2c0_bus_mutex_attr);
  g_i2c_bus_locks[INSTANCE_ONE] = osMutexNew(&i2c1_bus_mutex_attr);
  g_imu_ctx[0].lock = osMutexNew(&imu0_mutex_attr);
  g_imu_ctx[1].lock = osMutexNew(&imu1_mutex_attr);
  g_imu_ctx[0].dma_sem = osSemaphoreNew(1U, 0U, &imu0_dma_sem_attr);
  g_imu_ctx[1].dma_sem = osSemaphoreNew(1U, 0U, &imu1_dma_sem_attr);

  g_imu_ctx[0].thread_id = osThreadNew(imu_pair_task_entry, NULL, &imu_pair_attr);
  g_imu_ctx[1].thread_id = g_imu_ctx[0].thread_id;

  if (g_imu_ctx[0].thread_id == NULL) {
    printf("IMU pair task create failed\r\n");
    return;
  }

  g_dual_tasks_started = 1U;
}

uint8_t lsm6dso_get_latest_sample(uint8_t imu_index, lsm6dso_imu_sample_t *out)
{
  imu_task_ctx_t *ctx;
  uint16_t latest_index;

  if ((imu_index >= (sizeof(g_imu_ctx) / sizeof(g_imu_ctx[0]))) || (out == NULL)) {
    return 0U;
  }

  ctx = &g_imu_ctx[imu_index];
  if (ctx->lock != NULL) {
    (void)osMutexAcquire(ctx->lock, osWaitForever);
  }

  if ((ctx->latest_valid == 0U) || (ctx->ring_count == 0U)) {
    if (ctx->lock != NULL) {
      (void)osMutexRelease(ctx->lock);
    }
    return 0U;
  }

  latest_index = (uint16_t)((ctx->ring_head + LSM6DSO_IMU_STORAGE_CAPACITY - 1U)
                            % LSM6DSO_IMU_STORAGE_CAPACITY);
  lsm6dso_expand_stored_sample(ctx, &ctx->ring[latest_index], out);

  if (ctx->lock != NULL) {
    (void)osMutexRelease(ctx->lock);
  }

  return 1U;
}

uint16_t lsm6dso_copy_recent_samples(uint8_t imu_index,
                                     lsm6dso_imu_sample_t *out,
                                     uint16_t max_samples)
{
  imu_task_ctx_t *ctx;
  uint16_t n;
  uint16_t start;

  if ((imu_index >= (sizeof(g_imu_ctx) / sizeof(g_imu_ctx[0]))) ||
      (out == NULL) ||
      (max_samples == 0U)) {
    return 0U;
  }

  ctx = &g_imu_ctx[imu_index];
  if (ctx->lock != NULL) {
    (void)osMutexAcquire(ctx->lock, osWaitForever);
  }

  n = ctx->ring_count;
  if (n > max_samples) {
    n = max_samples;
  }

  start = (uint16_t)((ctx->ring_head + LSM6DSO_IMU_STORAGE_CAPACITY - n)
                     % LSM6DSO_IMU_STORAGE_CAPACITY);
  for (uint16_t i = 0U; i < n; i++) {
    uint16_t idx = (uint16_t)((start + i) % LSM6DSO_IMU_STORAGE_CAPACITY);
    lsm6dso_expand_stored_sample(ctx, &ctx->ring[idx], &out[i]);
  }

  if (ctx->lock != NULL) {
    (void)osMutexRelease(ctx->lock);
  }

  return n;
}

uint8_t lsm6dso_get_stored_sample(uint8_t imu_index,
                                  uint16_t sample_index,
                                  lsm6dso_imu_sample_t *out)
{
  imu_task_ctx_t *ctx;
  uint16_t start;
  uint16_t idx;

  if ((imu_index >= (sizeof(g_imu_ctx) / sizeof(g_imu_ctx[0]))) || (out == NULL)) {
    return 0U;
  }

  ctx = &g_imu_ctx[imu_index];
  if (ctx->lock != NULL) {
    (void)osMutexAcquire(ctx->lock, osWaitForever);
  }

  if (sample_index >= ctx->ring_count) {
    if (ctx->lock != NULL) {
      (void)osMutexRelease(ctx->lock);
    }
    return 0U;
  }

  start = (uint16_t)((ctx->ring_head + LSM6DSO_IMU_STORAGE_CAPACITY - ctx->ring_count)
                     % LSM6DSO_IMU_STORAGE_CAPACITY);
  idx = (uint16_t)((start + sample_index) % LSM6DSO_IMU_STORAGE_CAPACITY);
  lsm6dso_expand_stored_sample(ctx, &ctx->ring[idx], out);

  if (ctx->lock != NULL) {
    (void)osMutexRelease(ctx->lock);
  }

  return 1U;
}

uint8_t lsm6dso_get_storage_stats(uint8_t imu_index,
                                  lsm6dso_imu_storage_stats_t *out)
{
  imu_task_ctx_t *ctx;

  if ((imu_index >= (sizeof(g_imu_ctx) / sizeof(g_imu_ctx[0]))) || (out == NULL)) {
    return 0U;
  }

  ctx = &g_imu_ctx[imu_index];
  if (ctx->lock != NULL) {
    (void)osMutexAcquire(ctx->lock, osWaitForever);
  }

  out->total_samples = ctx->total_samples;
  out->stored_samples = ctx->ring_count;
  out->overwritten_samples = ctx->overwritten_samples;
  out->storage_full = (ctx->ring_count >= LSM6DSO_IMU_STORAGE_CAPACITY) ? 1U : 0U;

  if (ctx->lock != NULL) {
    (void)osMutexRelease(ctx->lock);
  }

  return 1U;
}

uint8_t lsm6dso_clear_stored_samples(uint8_t imu_index)
{
  imu_task_ctx_t *ctx;

  if (imu_index >= (sizeof(g_imu_ctx) / sizeof(g_imu_ctx[0]))) {
    return 0U;
  }

  ctx = &g_imu_ctx[imu_index];
  if (ctx->lock != NULL) {
    (void)osMutexAcquire(ctx->lock, osWaitForever);
  }

  ctx->ring_head = 0U;
  ctx->ring_count = 0U;
  ctx->latest_valid = 0U;
  ctx->total_samples = 0U;
  ctx->overwritten_samples = 0U;
  ctx->storage_base_tick = 0U;
  (void)memset(ctx->ring, 0, sizeof(ctx->ring));

  if (ctx->lock != NULL) {
    (void)osMutexRelease(ctx->lock);
  }

  return 1U;
}

void lsm6dso_set_storage_enabled(uint8_t enabled)
{
  g_storage_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t lsm6dso_get_storage_enabled(void)
{
  return g_storage_enabled;
}

void lsm6dso_set_pair_sampling_paused(uint8_t paused)
{
  g_pair_sampling_paused = (paused != 0U) ? 1U : 0U;
}

uint8_t lsm6dso_get_pair_sampling_paused(void)
{
  return g_pair_sampling_paused;
}

uint8_t lsm6dso_pair_task_is_ready(void)
{
  return g_pair_task_ready;
}

void i2c_leader_example_init(void)
{
  i2c_dual_imu_tasks_start();
}

void i2c_leader_example_process_action(void *pvParameters)
{
  (void)pvParameters;
  osThreadExit();
}
