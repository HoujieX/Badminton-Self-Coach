/***************************************************************************/ /**
 * @file i2c_leader_example.h
 * @brief I2C Leader Blocking example functions
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
#ifndef I2C_LEADER_EXAMPLE_H_
#define I2C_LEADER_EXAMPLE_H_

#include <stdint.h>
#include "sl_si91x_i2c.h"

// -----------------------------------------------------------------------------
// Prototypes
typedef struct {
  int16_t gx_raw;
  int16_t gy_raw;
  int16_t gz_raw;
  int16_t ax_raw;
  int16_t ay_raw;
  int16_t az_raw;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float ax_mps2;
  float ay_mps2;
  float az_mps2;
} lsm6dso_imu_data_t;

typedef struct {
  uint32_t sequence;
  uint32_t tick_ms;
  lsm6dso_imu_data_t data;
} lsm6dso_imu_sample_t;

#define LSM6DSO_IMU_STORAGE_CAPACITY 3200U

typedef struct {
  uint32_t total_samples;
  uint32_t stored_samples;
  uint32_t overwritten_samples;
  uint8_t storage_full;
} lsm6dso_imu_storage_stats_t;

/***************************************************************************/ /**
 * Read one full IMU sample from OUTX_L_G (12 bytes) and convert to engineering
 * units (gyro dps, accel m/s^2).
 ******************************************************************************/
sl_i2c_status_t lsm6dso_read_imu_data(sl_i2c_instance_t inst,
                                      uint8_t addr,
                                      lsm6dso_imu_data_t *out);

/***************************************************************************/ /**
 * Start the coordinated dual-IMU sampling task:
 * - IMU0: INSTANCE_ZERO, addr 0x6B
 * - IMU1: INSTANCE_ONE,  addr 0x6A
 ******************************************************************************/
void i2c_dual_imu_tasks_start(void);

void i2c_leader_example_init(void);
void i2c_leader_example_process_action(void *pvParameters);

/***************************************************************************/ /**
 * Get latest sample for IMU index (0 or 1). Returns 1 on success, 0 on fail.
 ******************************************************************************/
uint8_t lsm6dso_get_latest_sample(uint8_t imu_index, lsm6dso_imu_sample_t *out);

/***************************************************************************/ /**
 * Copy up to max_samples recent samples (oldest->newest) for IMU index (0 or 1).
 * Return value is number of copied samples.
 ******************************************************************************/
uint16_t lsm6dso_copy_recent_samples(uint8_t imu_index,
                                     lsm6dso_imu_sample_t *out,
                                     uint16_t max_samples);

/***************************************************************************/ /**
 * Get one stored sample by oldest-first index for IMU index (0 or 1).
 ******************************************************************************/
uint8_t lsm6dso_get_stored_sample(uint8_t imu_index,
                                  uint16_t sample_index,
                                  lsm6dso_imu_sample_t *out);

/***************************************************************************/ /**
 * Get storage counters for IMU index (0 or 1). Returns 1 on success, 0 on fail.
 ******************************************************************************/
uint8_t lsm6dso_get_storage_stats(uint8_t imu_index,
                                  lsm6dso_imu_storage_stats_t *out);

/***************************************************************************/ /**
 * Clear stored samples and counters for IMU index (0 or 1).
 ******************************************************************************/
uint8_t lsm6dso_clear_stored_samples(uint8_t imu_index);

/***************************************************************************/ /**
 * Enable or disable writing newly-read samples into RAM storage.
 ******************************************************************************/
void lsm6dso_set_storage_enabled(uint8_t enabled);

uint8_t lsm6dso_get_storage_enabled(void);
void lsm6dso_set_pair_sampling_paused(uint8_t paused);
uint8_t lsm6dso_get_pair_sampling_paused(void);
uint8_t lsm6dso_pair_task_is_ready(void);

uint8_t i2c_shared_bus_lock(sl_i2c_instance_t inst, uint32_t timeout_ms);
void i2c_shared_bus_unlock(sl_i2c_instance_t inst);

#endif /* I2C_LEADER_EXAMPLE_H_ */
