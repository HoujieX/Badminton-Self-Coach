/**************************************************************************/ /**
 * @file      button.h
 * @brief     Button task
 * @author    Nick McGill-Gardner
 * @date      2026/03/15
 ******************************************************************************/

#pragma once
#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 * Includes
 ******************************************************************************/
#include "sl_si91x_button_instances.h"
#include "led.h"
//#include "systick.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define BUTTON_PRESS_TIME_MS 1000   ///< Time, in ms, that a button must be pressed to count as a press.


/******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/
//! Button state variable definition
typedef enum button_control_states {
    BUTTON_STATE_RELEASED,      ///< BUTTON IS RELEASED
    BUTTON_STATE_PRESSED,       ///< BUTTON IS PRESSED
    BUTTON_STATE_MAX            ///< Max number of states.
} button_control_states;


/******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
void button_init_task(void);
void button_process_task(void);


#ifdef __cplusplus
}
#endif