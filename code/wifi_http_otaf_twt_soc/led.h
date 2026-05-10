/**************************************************************************/ /**
 * @file      led.h
 * @brief     LED task
 * @author    Nick McGill-Gardner
 * @date      2026/03/15
 ******************************************************************************/

#ifndef LED_H
#define LED_H

/******************************************************************************
* Includes
******************************************************************************/
#include "rsi_ccp_user_config.h"
#include "sl_si91x_led_instances.h"
#include "rsi_debug.h"
#include "sl_si91x_led.h"
//#include "systick.h"

/******************************************************************************
* Defines
******************************************************************************/
#define LED_ONOFF_TIME_2000MS	2000	//Time, in ms, of an LED being ON/OFF for it to blink at 0.25Hz
#define LED_ONOFF_TIME_1000MS	1000	//Time, in ms, of an LED being ON/OFF for it to blink at 0.5Hz
#define LED_ONOFF_TIME_500MS	500		//Time, in ms, of an LED being ON/OFF for it to blink at 1Hz


/******************************************************************************
* Structures and Enumerations
******************************************************************************/
//! LED state variable definition
typedef enum led_control_state
{
	LED_INIT_STATE, ///<Describes the LED initial state
	LED_ON_STATE, ///<LED is ON
	LED_OFF_STATE, ///LED is OFF
	LED_MAX_STATES ///Max number of states.
}led_control_state;

//! Possible frequencies of LEDs
typedef enum led_frequencies
{
	LED_FREQ_1, ///<LED Frequency 1
	LED_FREQ_2, ///<LED Frequency 2
	LED_FREQ_3, ///LED Frequency 3
	LED_MAX_FREQ_NUM ///Max number of frequences allowed.
}led_frequencies;


/******************************************************************************
* Global Function Declaration
******************************************************************************/
void led_init_task(void);
void led_process_task(void);
uint32_t led_get_target_time_ms(led_frequencies freq);
void led_change_frequency(void);


#endif // BLINK_H
