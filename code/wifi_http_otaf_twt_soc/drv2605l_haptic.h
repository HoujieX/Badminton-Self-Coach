/***************************************************************************/ /**
 * @file drv2605l_haptic.h
 * @brief DRV2605L haptic motor task interface.
 ******************************************************************************/
#ifndef DRV2605L_HAPTIC_H_
#define DRV2605L_HAPTIC_H_

#include <stdint.h>
#include "sl_si91x_i2c.h"

#ifndef INSTANCE_ZERO
#define INSTANCE_ZERO 0
#endif

#ifndef INSTANCE_ONE
#define INSTANCE_ONE 1
#endif

#define DRV2605L_HAPTIC_I2C_INSTANCE INSTANCE_ZERO
#define DRV2605L_HAPTIC_I2C_ADDR     0x5AU
#define DRV2605L_HAPTIC_STARTUP_TEST 1U
#define DRV2605L_HAPTIC_TEST_EFFECT  1U
#define DRV2605L_HAPTIC_TEST_RTP_MAGNITUDE 180U
#define DRV2605L_HAPTIC_TEST_RTP_DURATION_MS 500U
#define DRV2605L_HAPTIC_DEBUG        1U
#define DRV2605L_ENABLE_PORT         SL_GPIO_PORT_B
#define DRV2605L_ENABLE_PIN          12U
#define DRV2605L_HAPTIC_INIT_I2C_DRIVER 1U

void drv2605l_haptic_task_start(void);
uint8_t drv2605l_haptic_play_effect(uint8_t effect_id);
uint8_t drv2605l_haptic_stop(void);
uint8_t drv2605l_haptic_set_realtime(uint8_t magnitude, uint16_t duration_ms);
uint8_t drv2605l_haptic_set_realtime_wait(uint8_t magnitude,
                                          uint16_t duration_ms,
                                          uint32_t timeout_ms);

#endif /* DRV2605L_HAPTIC_H_ */
