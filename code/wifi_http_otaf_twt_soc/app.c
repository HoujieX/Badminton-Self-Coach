/***************************************************************************/ /**
 * @file
 * @brief HTTP OTAF Example Application
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "cmsis_os2.h"
#include "sl_board_configuration.h"
#include "sl_net.h"
#include "sl_wifi_types.h"
#include <stdio.h>
#include <string.h>
#include "sl_wifi.h"
#include "sl_wifi_callback_framework.h"
#include "firmware_upgradation.h"
#include "sl_net_dns.h"
#include "sl_utility.h"
#include "sl_net_si91x.h"
#include "sl_net_wifi_types.h"

#include "sl_constants.h"
#include "sl_mqtt_client.h"


#include "sl_net_types.h"    
#include "sl_net_ip_types.h" 

#include "led.h"
#include "button.h"

#include "sl_si91x_usart.h"

#include "i2c_leader_example.h"
#include "drv2605l_haptic.h"
#include "app.h"
#include "ST7735.h"
#include "LCD_GFX.h"

extern const sl_button_t button_btn0;

// include certificates
#include "aws_starfield_ca.pem.h"
//#include "azure_baltimore_ca.pem.h"
#include "silabs_dgcert_ca.pem.h"

#include "cacert.pem.h"

#ifdef SLI_SI91X_MCU_INTERFACE
#include "sl_si91x_hal_soc_soft_reset.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define ST7735_GPIO_TEST_ONLY 0
#define APP_LCD_ENABLED      1
#define APP_GPIO_INIT_WITHOUT_LCD 1
#define APP_HAPTIC_DEBUG_ONLY 0
#define APP_HAPTIC_DEBUG_WITH_LCD 1
#define APP_HAPTIC_DEBUG_WITH_BUTTON 1
#define APP_IMU_ENABLED      1
#define APP_IMU_DEBUG_ONLY    0
#define APP_IMU_SERIAL_PRINT_ENABLED 0
#define APP_BUTTON_TEST_ONLY  0 
#define BUTTON_PUBLISH_MQTT_ON_PRESS 0
#define IMU_COUNTDOWN_MS      3000U
#define IMU_PRINT_WINDOW_MS   3000U
#define IMU_RECORD_TARGET_PAIRS 500U
#define IMU_RECORD_MAX_WINDOW_MS 6000U
#define IMU_HAPTIC_START_MAGNITUDE 140U
#define IMU_HAPTIC_START_MS        120U
#define IMU_HAPTIC_DONE_MAGNITUDE  190U
#define IMU_HAPTIC_DONE_MS         250U

/******************************************************
 *                    Constants
 ******************************************************/
//! Type of FW update

 #undef AWS_ENABLE
 #define AZURE_ENABLE 1

#define M4_FW_UPDATE       0
#define TA_FW_UPDATE       1
#define COMBINED_FW_UPDATE 2

//! Set FW update type
#define FW_UPDATE_TYPE TA_FW_UPDATE

//! Load certificate to device flash :
//! Certificate should be loaded once and need not be loaded for every boot up
#define LOAD_CERTIFICATE 1

// Macro to set specified bit position
#define BIT(a) ((uint32_t)1U << a)

//! Enable IPv6 set this bit in FLAGS, Default is IPv4
#define HTTPV6 BIT(3)

//! Set HTTPS_SUPPORT to use HTTPS feature
#define HTTPS_SUPPORT BIT(0)

//! Set HTTP_POST_DATA to use HTTP POST LARGE DATA feature
#define HTTP_POST_DATA BIT(5)

//! Set HTTP_V_1_1 to use HTTP version 1.1
#define HTTP_V_1_1 BIT(6)

//! Enable user defined http content type in FLAGS
#define HTTP_USER_DEFINED_CONTENT_TYPE BIT(7)

//! Enable Server Name Indication for HTTPS servers that host multiple domains
#define HTTPS_SNI BIT(11)

// HTTP OTAF
#define HTTP_OTAF 2

//! set 1 for selecting SL_SI91X_HTTPS_CERTIFICATE_INDEX_1, set 2 for selecting SL_SI91X_HTTPS_CERTIFICATE_INDEX_2
#define CERTIFICATE_INDEX 0

#define DNS_TIMEOUT         20000
#define MAX_DNS_RETRY_COUNT 5
#define OTAF_TIMEOUT        600000
#ifdef AWS_ENABLE
//! for example select required flag bits,  Eg:(HTTPS_SUPPORT | HTTPV6 | HTTP_USER_DEFINED_CONTENT_TYPE)
#define FLAGS HTTPS_SUPPORT
//! Server port number
#define HTTP_PORT 443
//! Server URL
#if (FW_UPDATE_TYPE == TA_FW_UPDATE)
#define HTTP_URL "rps/firmware.rps"
#else
#define HTTP_URL "isp.bin"
#endif
//! Server Hostname
char *hostname = "otafaws.s3.ap-south-1.amazonaws.com";
//! set HTTP extended header
//! if NULL , driver fills default extended header
#define HTTP_EXTENDED_HEADER NULL
//! set Username
#define USERNAME ""
//! set Password
#define PASSWORD    ""
#define SERVER_NAME "AWS Server"

#elif AZURE_ENABLE
//! for example select required flag bits,  Eg:(HTTPS_SUPPORT | HTTPV6 | HTTP_USER_DEFINED_CONTENT_TYPE)
#define FLAGS                (HTTPS_SUPPORT | HTTP_V_1_1 | HTTPS_SNI)
//! Server port number
#define HTTP_PORT            443
//! Server URL
#define HTTP_URL             "firmware/wifi_http_otaf_twt_soc.rps"
//! Server Hostname
char *hostname = "masterr869.blob.core.windows.net";
//! set HTTP extended header
#define HTTP_EXTENDED_HEADER NULL
//! set Username
#define USERNAME             ""
//! set Password
#define PASSWORD             ""
#define SERVER_NAME          "AZURE Server"
#else
#define FLAGS                  0
//! Server port number
#define HTTP_PORT              80
//! HTTP Server IP address.
#define HTTP_SERVER_IP_ADDRESS "192.168.0.100"
//! HTTP resource name
#if (FW_UPDATE_TYPE == TA_FW_UPDATE)
#define HTTP_URL "rps/firmware.rps"
#else
#define HTTP_URL "isp.bin"
#endif
//! set HTTP hostname
#define HTTP_HOSTNAME        "192.168.0.100"
char *hostname = HTTP_HOSTNAME;
//! set HTTP extended header
//! if NULL , driver fills default extended header
#define HTTP_EXTENDED_HEADER NULL
//! set HTTP hostname
#define USERNAME             "admin"
//! set HTTP hostname
#define PASSWORD             "admin"
#define SERVER_NAME          "Local Apache Server"
#endif

/******************************************************
 *               Constants for MQTT Client
 ******************************************************/

#ifdef SLI_SI91X_ENABLE_IPV6
#define MQTT_BROKER_IP "20.25.211.36"
#else
#define MQTT_BROKER_IP "20.25.211.36"
#endif

#define MQTT_BROKER_PORT 1883

#define CLIENT_PORT 1

#define CLIENT_ID "WISECONNECT-SDK-MQTT-CLIENT-ID-123"

#define TOPIC_TO_BE_SUBSCRIBED "OTA"
#define QOS_OF_SUBSCRIPTION    SL_MQTT_QOS_LEVEL_1


#define PUBLISH_TOPIC          "REMOTE-LED"
#define IMU_MQTT_TOPIC         "DATA"
#define PUBLISH_MESSAGE        "ON"
#define OTA_TRIGGER_MESSAGE    "do ota"

#define QOS_OF_PUBLISH_MESSAGE 0

#define IS_DUPLICATE_MESSAGE 0
#define IS_MESSAGE_RETAINED  1
#define IS_CLEAN_SESSION     1

#define LAST_WILL_TOPIC       "WISECONNECT-SDK-MQTT-CLIENT-LAST-WILL"
#define LAST_WILL_MESSAGE     "WISECONNECT-SDK-MQTT-CLIENT has been disconnect from network"
#define QOS_OF_LAST_WILL      1
#define IS_LAST_WILL_RETAINED 1

#define ENCRYPT_CONNECTION     0
#define KEEP_ALIVE_INTERVAL    2000
#define MQTT_CONNECT_TIMEOUT   5000
#define MQTT_KEEPALIVE_RETRIES 0

#define SEND_CREDENTIALS 0
#define IMU_MQTT_PUBLISH_PERIOD_MS 200U
#define IMU_MQTT_PAYLOAD_SIZE      512U
#define IMU_MQTT_BATCH_DELAY_MS    20U
#define IMU_MQTT_PUBLISH_TIMEOUT_MS 3000U
#define IMU_MQTT_MAX_PUBLISH_PAIRS IMU_RECORD_TARGET_PAIRS
#define IMU_MQTT_RETRY_ON_FAIL     0




/******************************************************
 *               Variable Definitions for wifi 
 ******************************************************/
const osThreadAttr_t thread_attributes = {
  .name       = "app",
  .attr_bits  = 0,
  .cb_mem     = 0,
  .cb_size    = 0,
  .stack_mem  = 0,
  .stack_size = 3072,
  .priority   = osPriorityLow,
  .tz_module  = 0,
  .reserved   = 0,
};

static const sl_wifi_device_configuration_t station_init_configuration = {
  .boot_option = LOAD_NWP_FW,
  .mac_address = NULL,
  .band        = SL_SI91X_WIFI_BAND_2_4GHZ,
  .region_code = US,
  .boot_config = { .oper_mode              = SL_SI91X_CLIENT_MODE,
                   .coex_mode              = SL_SI91X_WLAN_ONLY_MODE,
                   .feature_bit_map        = (SL_SI91X_FEAT_SECURITY_PSK | SL_SI91X_FEAT_AGGREGATION),
                   .tcp_ip_feature_bit_map = (SL_SI91X_TCP_IP_FEAT_DHCPV4_CLIENT | SL_SI91X_TCP_IP_FEAT_HTTP_CLIENT
                                              | SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID | SL_SI91X_TCP_IP_FEAT_SSL
                                              | SL_SI91X_TCP_IP_FEAT_DNS_CLIENT),
                   .custom_feature_bit_map = SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID,
                   .ext_custom_feature_bit_map =
                     (SL_SI91X_EXT_FEAT_XTAL_CLK | SL_SI91X_EXT_FEAT_UART_SEL_FOR_DEBUG_PRINTS | MEMORY_CONFIG
#if defined(SLI_SI917) || defined(SLI_SI915)
                      | SL_SI91X_EXT_FEAT_FRONT_END_SWITCH_PINS_ULP_GPIO_4_5_0
#endif
                      ),
                   .bt_feature_bit_map = 0,
                   .ext_tcp_ip_feature_bit_map =
                     (SL_SI91X_EXT_FEAT_HTTP_OTAF_SUPPORT | SL_SI91X_EXT_TCP_IP_SSL_16K_RECORD
                      | SL_SI91X_CONFIG_FEAT_EXTENTION_VALID | SL_SI91X_EXT_EMB_MQTT_ENABLE),
                   .ble_feature_bit_map     = 0,
                   .ble_ext_feature_bit_map = 0,
                   .config_feature_bit_map  = 0 }
};

//! Enumeration for states in application
typedef enum {
  WLAN_INITIAL_STATE = 0,
  WLAN_UNCONNECTED_STATE,
  CALIBRATION,
  DATA_COLLECTION,
  MODEL_ANALYTICS,
  ANALYZED_DATA_PUBLISHING,
  DONE,
} wlan_app_state_t;

volatile wlan_app_state_t app_state;
volatile bool response               = false;
volatile sl_status_t callback_status = SL_STATUS_OK;


/******************************************************
 *               Variable Definitions for MQTT Client 
 ******************************************************/

sl_mqtt_client_t client = { 0 };

uint8_t is_execution_completed = 0;

sl_mqtt_client_credentials_t *client_credentails = NULL;

sl_mqtt_client_configuration_t mqtt_client_configuration = { .is_clean_session = IS_CLEAN_SESSION,
                                                             .client_id        = (uint8_t *)CLIENT_ID,
                                                             .client_id_length = strlen(CLIENT_ID),
#if ENCRYPT_CONNECTION
                                                             .tls_flags = SL_MQTT_TLS_ENABLE | SL_MQTT_TLS_TLSV_1_2
                                                                          | SL_MQTT_TLS_CERT_INDEX_1,
#endif
                                                             .client_port = CLIENT_PORT };

sl_mqtt_broker_t mqtt_broker_configuration = {
  .port                    = MQTT_BROKER_PORT,
  .is_connection_encrypted = ENCRYPT_CONNECTION,
  .connect_timeout         = MQTT_CONNECT_TIMEOUT,
  .keep_alive_interval     = KEEP_ALIVE_INTERVAL,
  .keep_alive_retries      = MQTT_KEEPALIVE_RETRIES,
};

sl_mqtt_client_message_t message_to_be_published = {
  .qos_level            = QOS_OF_PUBLISH_MESSAGE,
  .is_retained          = IS_MESSAGE_RETAINED,
  .is_duplicate_message = IS_DUPLICATE_MESSAGE,
  .topic                = (uint8_t *)PUBLISH_TOPIC,
  .topic_length         = strlen(PUBLISH_TOPIC),
  .content              = (uint8_t *)PUBLISH_MESSAGE,
  .content_length       = strlen(PUBLISH_MESSAGE),
};

sl_mqtt_client_last_will_message_t last_will_message = {
  .is_retained         = IS_LAST_WILL_RETAINED,
  .will_qos_level      = QOS_OF_LAST_WILL,
  .will_topic          = (uint8_t *)LAST_WILL_TOPIC,
  .will_topic_length   = strlen(LAST_WILL_TOPIC),
  .will_message        = (uint8_t *)LAST_WILL_MESSAGE,
  .will_message_length = strlen(LAST_WILL_MESSAGE),
};




/******************************************************
 *               Function Declarations
 ******************************************************/
void application_start(const void *unused);
sl_status_t http_otaf_app();
static sl_status_t run_http_otaf_update(uint16_t flags);
static sl_status_t http_fw_update_response_handler(sl_wifi_event_t event,
                                                   uint16_t *data,
                                                   uint32_t data_length,
                                                   void *arg);

#if LOAD_CERTIFICATE
static sl_status_t clear_and_load_certificates_in_flash(void);
// static void v_LCD_Test_Task(void *pvParameters);
#endif
#if APP_LCD_ENABLED
static void lcd_init_once(void);
static void lcd_draw_text_line_scaled_fast(uint8_t x_start,
                                           uint8_t y_start,
                                           const char *text,
                                           uint8_t scale,
                                           uint8_t glyph_step,
                                           uint8_t space_step,
                                           uint16_t fg,
                                           uint16_t bg);
static void lcd_show_start_prompt(void);
static void v_LCD_Notify_Task(void *pvParameters);
#endif
#if APP_IMU_ENABLED
static void v_IMU_MQTT_Task(void *pvParameters);
#endif

typedef enum {
  IMU_PIPELINE_IDLE = 0,
  IMU_PIPELINE_RECORDING,
  IMU_PIPELINE_READY_TO_SEND,
  IMU_PIPELINE_SENDING
} imu_pipeline_state_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

void mqtt_client_message_handler(void *client, sl_mqtt_client_message_t *message, void *context);
void mqtt_client_event_handler(void *client, sl_mqtt_client_event_t event, void *event_data, void *context);
void mqtt_client_error_event_handler(void *client, sl_mqtt_client_error_status_t *error);
void mqtt_client_cleanup();
void print_char_buffer(char *buffer, uint32_t buffer_length);
sl_status_t mqtt_example();


/******************************************************
 *               Function Definitions
 ******************************************************/

volatile uint8_t led_flag = 1;
volatile uint8_t imu_print_enabled = 0U;
static volatile uint8_t mqtt_connected = 0U;
static volatile uint8_t imu_mqtt_publish_busy = 0U;
static volatile uint8_t ota_update_requested = 0U;
static volatile imu_pipeline_state_t imu_pipeline_state = IMU_PIPELINE_IDLE;
#if APP_IMU_ENABLED
static char mqttData[IMU_MQTT_PAYLOAD_SIZE];
static sl_mqtt_client_message_t imu_message_to_be_published = {
  .qos_level            = QOS_OF_PUBLISH_MESSAGE,
  .is_retained          = 0,
  .is_duplicate_message = IS_DUPLICATE_MESSAGE,
  .topic                = (uint8_t *)IMU_MQTT_TOPIC,
  .topic_length         = sizeof(IMU_MQTT_TOPIC) - 1U,
  .content              = (uint8_t *)mqttData,
  .content_length       = 0U,
};
#endif
#if APP_LCD_ENABLED
static bool lcd_initialized = false;
#endif
// 0: no request, 1: draw green bar, 2: clear bar
static volatile uint8_t lcd_status_request = 0U;
static volatile uint32_t button_irq_count = 0U;

// Button interrupt service routine for button BTN0.
void sl_si91x_button_isr(uint8_t pin, int8_t state)
{
  (void)state;
  if (pin == button_btn0.pin) {
    button_irq_count++;
  }
}

// Button task handles press debounce and starts IMU recording or MQTT publishes.
void v_BUTTON_Task(void *pvParameters)
{
  (void)pvParameters;

#if BUTTON_PUBLISH_MQTT_ON_PRESS
  sl_mqtt_client_message_t button_publish_message = {
  .qos_level            = QOS_OF_PUBLISH_MESSAGE,
  .is_retained          = IS_MESSAGE_RETAINED,
  .is_duplicate_message = IS_DUPLICATE_MESSAGE,
  .topic                = (uint8_t *)PUBLISH_TOPIC,
  .topic_length         = 10,
  .content              = (uint8_t *)"ON",
  .content_length       = 2,
  };
#endif
  uint32_t handled_irq_count = 0U;
  uint32_t last_handled_tick = 0U;
  const uint32_t debounce_ms = 120U;
  typedef enum {
    BUTTON_IMU_IDLE = 0,
    BUTTON_IMU_COUNTDOWN,
    BUTTON_IMU_PRINTING
  } button_imu_state_t;
  button_imu_state_t button_imu_state = BUTTON_IMU_IDLE;
  uint32_t state_start_tick = 0U;

  printf("BUTTON CFG: port=%u pin=%u (interrupt mode)\r\n",
         (unsigned int)button_btn0.port,
         (unsigned int)button_btn0.pin);
  printf("BUTTON TASK: trigger=pressed-event, debounce=%lums\r\n", (unsigned long)debounce_ms);

  handled_irq_count = button_irq_count;

  while (1) {
    uint32_t now = osKernelGetTickCount();

    uint32_t irq_count_snapshot = button_irq_count;
    if (irq_count_snapshot != handled_irq_count) {
      handled_irq_count = irq_count_snapshot;

      if ((now - last_handled_tick) >= debounce_ms) {
        last_handled_tick = now;
        if (imu_pipeline_state != IMU_PIPELINE_IDLE) {
          printf("BUTTON: ignored, IMU pipeline busy state=%u\r\n",
                 (unsigned int)imu_pipeline_state);
          continue;
        }
#if !APP_IMU_ENABLED
        printf("BUTTON: IMU disabled by APP_IMU_ENABLED=0\r\n");
        continue;
#endif
        /* One button press triggers the following flow: 3s countdown -> fixed sample recording */
        button_imu_state = BUTTON_IMU_COUNTDOWN;
        state_start_tick = now;
        imu_print_enabled = 0U;
        lcd_status_request = 2U;
        lsm6dso_set_storage_enabled(0U);
        (void)lsm6dso_clear_stored_samples(0U);
        (void)lsm6dso_clear_stored_samples(1U);
        printf("BUTTON: trigger accepted, IMU record countdown 3s, target=%u pairs\r\n",
               (unsigned int)IMU_RECORD_TARGET_PAIRS);
#if BUTTON_PUBLISH_MQTT_ON_PRESS
        if (mqtt_connected != 0U) {
          button_publish_message.content = (uint8_t *)"OFF";
          button_publish_message.content_length = 3U;
          (void)sl_mqtt_client_publish(&client,
                                       &button_publish_message,
                                       0,
                                       &button_publish_message);
        }
#endif
      }
    }

    if (button_imu_state == BUTTON_IMU_COUNTDOWN) {
      if ((now - state_start_tick) >= IMU_COUNTDOWN_MS) {
        button_imu_state = BUTTON_IMU_PRINTING;
        state_start_tick = now;
        imu_print_enabled = APP_IMU_SERIAL_PRINT_ENABLED ? 1U : 0U;
        lcd_status_request = 1U;
        (void)drv2605l_haptic_set_realtime_wait(IMU_HAPTIC_START_MAGNITUDE,
                                                IMU_HAPTIC_START_MS,
                                                IMU_HAPTIC_START_MS + 1000U);
        now = osKernelGetTickCount();
        state_start_tick = now;
        imu_pipeline_state = IMU_PIPELINE_RECORDING;
        lsm6dso_set_storage_enabled(1U);
        printf("BUTTON: countdown done, IMU record ON until %u paired samples\r\n",
               (unsigned int)IMU_RECORD_TARGET_PAIRS);
#if BUTTON_PUBLISH_MQTT_ON_PRESS
        if (mqtt_connected != 0U) {
          button_publish_message.content = (uint8_t *)"ON";
          button_publish_message.content_length = 2U;
          (void)sl_mqtt_client_publish(&client,
                                       &button_publish_message,
                                       0,
                                       &button_publish_message);
        }
#endif
      }
    } else if (button_imu_state == BUTTON_IMU_PRINTING) {
      lsm6dso_imu_storage_stats_t imu0_stats;
      lsm6dso_imu_storage_stats_t imu1_stats;
      uint32_t imu0_count = 0U;
      uint32_t imu1_count = 0U;
      uint32_t pair_count;
      uint32_t elapsed_ms;
      uint32_t pair_rate_hz;
      uint8_t target_reached;
      uint8_t max_window_reached;

      (void)lsm6dso_get_storage_stats(0U, &imu0_stats);
      (void)lsm6dso_get_storage_stats(1U, &imu1_stats);
      imu0_count = imu0_stats.stored_samples;
      imu1_count = imu1_stats.stored_samples;
      pair_count = (imu0_count < imu1_count) ? imu0_count : imu1_count;
      target_reached = (pair_count >= IMU_RECORD_TARGET_PAIRS) ? 1U : 0U;
      max_window_reached = ((now - state_start_tick) >= IMU_RECORD_MAX_WINDOW_MS) ? 1U : 0U;

      if ((target_reached != 0U) || (max_window_reached != 0U)) {
        elapsed_ms = now - state_start_tick;
        pair_rate_hz = (elapsed_ms > 0U) ? ((pair_count * 1000U) / elapsed_ms) : 0U;
        button_imu_state = BUTTON_IMU_IDLE;
        imu_print_enabled = 0U;
        lcd_status_request = 2U;
        lsm6dso_set_storage_enabled(0U);
        imu_pipeline_state = IMU_PIPELINE_READY_TO_SEND;
        (void)drv2605l_haptic_set_realtime_wait(IMU_HAPTIC_DONE_MAGNITUDE,
                                                IMU_HAPTIC_DONE_MS,
                                                IMU_HAPTIC_DONE_MS + 1000U);
        printf("BUTTON: IMU record ended, imu0=%lu imu1=%lu pairs=%lu elapsed=%lums rate=%luHz reason=%s\r\n",
               (unsigned long)imu0_count,
               (unsigned long)imu1_count,
               (unsigned long)pair_count,
               (unsigned long)elapsed_ms,
               (unsigned long)pair_rate_hz,
               (target_reached != 0U) ? "target" : "timeout");
#if BUTTON_PUBLISH_MQTT_ON_PRESS
        if (mqtt_connected != 0U) {
          button_publish_message.content = (uint8_t *)"OFF";
          button_publish_message.content_length = 3U;
          (void)sl_mqtt_client_publish(&client,
                                       &button_publish_message,
                                       0,
                                       &button_publish_message);
        }
#endif
      }
    }

    osDelay(10);
  }
}

#if APP_LCD_ENABLED
// LCD notify task updates a status bar on the display when requested.
static void v_LCD_Notify_Task(void *pvParameters)
{
  (void)pvParameters;

  while (1) {
    if (lcd_status_request == 1U) {
      lcd_status_request = 0U;
      LCD_drawBlock(0U, 0U, (uint8_t)(LCD_WIDTH - 1U), 15U, GREEN);
    } else if (lcd_status_request == 2U) {
      lcd_status_request = 0U;
      LCD_drawBlock(0U, 0U, (uint8_t)(LCD_WIDTH - 1U), 15U, BLACK);
    }
    osDelay(20);
  }
}
#endif

#if APP_IMU_ENABLED
// IMU MQTT task sends recorded IMU data to the MQTT broker after recording completes.
static void v_IMU_MQTT_Task(void *pvParameters)
{
  (void)pvParameters;

  while (1) {
    lsm6dso_imu_storage_stats_t imu0_stats;
    lsm6dso_imu_storage_stats_t imu1_stats;
    uint16_t imu0_count;
    uint16_t imu1_count;
    uint16_t pair_count;
    uint8_t completed = 1U;
    uint8_t start_marker_pending = 0U;
    uint16_t publish_accepted_count = 0U;
    uint16_t publish_timeout_count = 0U;
    uint16_t publish_failed_count = 0U;

    if (imu_pipeline_state != IMU_PIPELINE_READY_TO_SEND) {
      osDelay(50U);
      continue;
    }

    if (mqtt_connected == 0U) {
      osDelay(100U);
      continue;
    }

    if (imu_mqtt_publish_busy != 0U) {
      osDelay(20U);
      continue;
    }

    imu_pipeline_state = IMU_PIPELINE_SENDING;

    (void)lsm6dso_get_storage_stats(0U, &imu0_stats);
    (void)lsm6dso_get_storage_stats(1U, &imu1_stats);
    imu0_count = (uint16_t)imu0_stats.stored_samples;
    imu1_count = (uint16_t)imu1_stats.stored_samples;
    pair_count = (imu0_count < imu1_count) ? imu0_count : imu1_count;
    if (pair_count > IMU_MQTT_MAX_PUBLISH_PAIRS) {
      pair_count = IMU_MQTT_MAX_PUBLISH_PAIRS;
    }

    printf("IMU MQTT send start: imu0=%u imu1=%u publish_pairs=%u\r\n",
           (unsigned int)imu0_count,
           (unsigned int)imu1_count,
           (unsigned int)pair_count);

    {
      int n = snprintf(mqttData,
                       sizeof(mqttData),
                       "{\"type\":\"imu_start\",\"pairs\":%u,"
                       "\"imu0\":{\"stored\":%lu,\"total\":%lu,\"overwritten\":%lu},"
                       "\"imu1\":{\"stored\":%lu,\"total\":%lu,\"overwritten\":%lu}}",
                       (unsigned int)pair_count,
                       (unsigned long)imu0_stats.stored_samples,
                       (unsigned long)imu0_stats.total_samples,
                       (unsigned long)imu0_stats.overwritten_samples,
                       (unsigned long)imu1_stats.stored_samples,
                       (unsigned long)imu1_stats.total_samples,
                       (unsigned long)imu1_stats.overwritten_samples);

      if ((n > 0) && ((uint32_t)n < sizeof(mqttData))) {
        sl_status_t publish_status;
        imu_message_to_be_published.content_length = (uint32_t)n;
        imu_mqtt_publish_busy = 1U;
        publish_status = sl_mqtt_client_publish(&client,
                                                &imu_message_to_be_published,
                                                0,
                                                &imu_message_to_be_published);
        if (publish_status == SL_STATUS_IN_PROGRESS) {
          start_marker_pending = 1U;
          /* Cleared by SL_MQTT_CLIENT_MESSAGE_PUBLISHED_EVENT. */
        } else if (publish_status == SL_STATUS_OK) {
          imu_mqtt_publish_busy = 0U;
        } else {
          imu_mqtt_publish_busy = 0U;
          completed = 0U;
          printf("IMU MQTT start publish failed status=0x%lx\r\n", publish_status);
        }
      } else {
        completed = 0U;
        printf("IMU MQTT start payload too large\r\n");
      }
    }

    for (uint16_t i = 0U; i < pair_count; i++) {
      lsm6dso_imu_sample_t imu0_sample;
      lsm6dso_imu_sample_t imu1_sample;
      int n;
      sl_status_t publish_status;
      uint32_t wait_start_tick;

      wait_start_tick = osKernelGetTickCount();
      while ((mqtt_connected != 0U) && (imu_mqtt_publish_busy != 0U)) {
        if ((osKernelGetTickCount() - wait_start_tick) > IMU_MQTT_PUBLISH_TIMEOUT_MS) {
          if ((i == 0U) && (start_marker_pending != 0U)) {
            start_marker_pending = 0U;
            imu_mqtt_publish_busy = 0U;
            publish_timeout_count++;
            printf("IMU MQTT start marker wait timeout, continue data publish\r\n");
          } else {
            imu_mqtt_publish_busy = 0U;
            publish_timeout_count++;
            printf("IMU MQTT publish wait timeout before idx=%u, continue\r\n", (unsigned int)i);
          }
          break;
        }
        osDelay(10U);
      }
      start_marker_pending = 0U;

      if (completed == 0U) {
        break;
      }

      if (mqtt_connected == 0U) {
        completed = 0U;
        printf("IMU MQTT send paused: broker disconnected\r\n");
        break;
      }

      if ((lsm6dso_get_stored_sample(0U, i, &imu0_sample) == 0U) ||
          (lsm6dso_get_stored_sample(1U, i, &imu1_sample) == 0U)) {
        completed = 0U;
        printf("IMU MQTT sample read failed at idx=%u\r\n", (unsigned int)i);
        break;
      }

      n = snprintf(mqttData,
                   sizeof(mqttData),
                   "{\"type\":\"imu\",\"idx\":%u,\"total\":%u,"
                   "\"i0\":{\"g\":[%.2f,%.2f,%.2f],\"a\":[%.2f,%.2f,%.2f]},"
                   "\"i1\":{\"g\":[%.2f,%.2f,%.2f],\"a\":[%.2f,%.2f,%.2f]}}",
                   (unsigned int)i,
                   (unsigned int)pair_count,
                   imu0_sample.data.gx_dps,
                   imu0_sample.data.gy_dps,
                   imu0_sample.data.gz_dps,
                   imu0_sample.data.ax_mps2,
                   imu0_sample.data.ay_mps2,
                   imu0_sample.data.az_mps2,
                   imu1_sample.data.gx_dps,
                   imu1_sample.data.gy_dps,
                   imu1_sample.data.gz_dps,
                   imu1_sample.data.ax_mps2,
                   imu1_sample.data.ay_mps2,
                   imu1_sample.data.az_mps2);

      if ((n <= 0) || ((uint32_t)n >= sizeof(mqttData))) {
        completed = 0U;
        printf("IMU MQTT payload too large at idx=%u\r\n", (unsigned int)i);
        break;
      }

      imu_message_to_be_published.content_length = (uint32_t)n;
      imu_mqtt_publish_busy = 1U;
      publish_status = sl_mqtt_client_publish(&client,
                                              &imu_message_to_be_published,
                                              0,
                                              &imu_message_to_be_published);
      if (publish_status == SL_STATUS_IN_PROGRESS) {
        publish_accepted_count++;
        /* Cleared by SL_MQTT_CLIENT_MESSAGE_PUBLISHED_EVENT. */
      } else if (publish_status == SL_STATUS_OK) {
        publish_accepted_count++;
        imu_mqtt_publish_busy = 0U;
      } else if (publish_status != SL_STATUS_OK) {
        imu_mqtt_publish_busy = 0U;
        completed = 0U;
        publish_failed_count++;
        printf("IMU MQTT publish failed idx=%u status=0x%lx\r\n",
               (unsigned int)i,
               publish_status);
        break;
      }

      if (((i + 1U) % 50U) == 0U) {
        printf("IMU MQTT progress: %u/%u accepted=%u timeout=%u failed=%u\r\n",
               (unsigned int)(i + 1U),
               (unsigned int)pair_count,
               (unsigned int)publish_accepted_count,
               (unsigned int)publish_timeout_count,
               (unsigned int)publish_failed_count);
      }

      osDelay(IMU_MQTT_BATCH_DELAY_MS);
    }

    uint32_t wait_start_tick = osKernelGetTickCount();
    while ((mqtt_connected != 0U) && (imu_mqtt_publish_busy != 0U)) {
      if ((osKernelGetTickCount() - wait_start_tick) > IMU_MQTT_PUBLISH_TIMEOUT_MS) {
        completed = 0U;
        imu_mqtt_publish_busy = 0U;
        printf("IMU MQTT final publish wait timeout\r\n");
        break;
      }
      osDelay(10U);
    }

    if ((completed != 0U) && (mqtt_connected != 0U)) {
      int n = snprintf(mqttData,
                       sizeof(mqttData),
                       "{\"type\":\"imu_end\",\"pairs\":%u,"
                       "\"imu0\":{\"stored\":%lu,\"total\":%lu,\"overwritten\":%lu},"
                       "\"imu1\":{\"stored\":%lu,\"total\":%lu,\"overwritten\":%lu}}",
                       (unsigned int)pair_count,
                       (unsigned long)imu0_stats.stored_samples,
                       (unsigned long)imu0_stats.total_samples,
                       (unsigned long)imu0_stats.overwritten_samples,
                       (unsigned long)imu1_stats.stored_samples,
                       (unsigned long)imu1_stats.total_samples,
                       (unsigned long)imu1_stats.overwritten_samples);

      if ((n > 0) && ((uint32_t)n < sizeof(mqttData))) {
        imu_message_to_be_published.content_length = (uint32_t)n;
        imu_mqtt_publish_busy = 1U;
        sl_status_t publish_status = sl_mqtt_client_publish(&client,
                                                            &imu_message_to_be_published,
                                                            0,
                                                            &imu_message_to_be_published);
        if (publish_status == SL_STATUS_IN_PROGRESS) {
          /* Cleared by SL_MQTT_CLIENT_MESSAGE_PUBLISHED_EVENT. */
        } else if (publish_status == SL_STATUS_OK) {
          imu_mqtt_publish_busy = 0U;
        } else {
          imu_mqtt_publish_busy = 0U;
          completed = 0U;
          printf("IMU MQTT end publish failed status=0x%lx\r\n", publish_status);
        }
      }

      wait_start_tick = osKernelGetTickCount();
      while ((mqtt_connected != 0U) && (imu_mqtt_publish_busy != 0U)) {
        if ((osKernelGetTickCount() - wait_start_tick) > IMU_MQTT_PUBLISH_TIMEOUT_MS) {
          completed = 0U;
          imu_mqtt_publish_busy = 0U;
          printf("IMU MQTT end publish wait timeout\r\n");
          break;
        }
        osDelay(10U);
      }
    }

#if IMU_MQTT_RETRY_ON_FAIL
    imu_pipeline_state = (completed != 0U) ? IMU_PIPELINE_IDLE : IMU_PIPELINE_READY_TO_SEND;
    printf("IMU MQTT send %s\r\n", (completed != 0U) ? "done" : "will retry");
#else
    imu_pipeline_state = IMU_PIPELINE_IDLE;
    printf("IMU MQTT send %s: accepted=%u timeout=%u failed=%u\r\n",
           (completed != 0U) ? "done" : "stopped",
           (unsigned int)publish_accepted_count,
           (unsigned int)publish_timeout_count,
           (unsigned int)publish_failed_count);
#endif
  }
}
#endif

// LED task toggles the status LED based on the current led_flag state.
void v_LED_Task(void *pvParameters)
{
  (void)pvParameters;
  sl_si91x_led_clear(led_led0.pin);

  while(1){
    if (led_flag  == 1) {
      sl_si91x_led_clear(led_led0.pin);
    } else {
      sl_si91x_led_set(led_led0.pin);
    }

    osDelay(300);
  }
}

#if APP_LCD_ENABLED
static void lcd_init_once(void)
{
  if (!lcd_initialized) {
    lcd_init();
    lcd_initialized = true;
  }
}

static void lcd_draw_text_line_scaled_fast(uint8_t x_start,
                                           uint8_t y_start,
                                           const char *text,
                                           uint8_t scale,
                                           uint8_t glyph_step,
                                           uint8_t space_step,
                                           uint16_t fg,
                                           uint16_t bg)
{
  uint16_t line_width = 0U;
  uint8_t i = 0U;
  uint8_t src_row = 0U;
  uint8_t py = 0U;

  while (text[i] != '\0') {
    line_width = (uint16_t)(line_width + ((text[i] == ' ') ? space_step : glyph_step));
    i++;
  }

  if (line_width == 0U) {
    return;
  }

  for (py = 0U; py < (uint8_t)(8U * scale); py++) {
    src_row = (uint8_t)(py / scale);
    LCD_setAddr(x_start,
                (uint8_t)(y_start + py),
                (uint8_t)(x_start + line_width - 1U),
                (uint8_t)(y_start + py));

    i = 0U;
    while (text[i] != '\0') {
      uint8_t step = (text[i] == ' ') ? space_step : glyph_step;
      uint8_t dx = 0U;
      for (dx = 0U; dx < step; dx++) {
        uint16_t color = bg;
        if ((text[i] != ' ') && (dx < (uint8_t)(5U * scale))) {
          uint8_t ch = (uint8_t)text[i];
          uint8_t row_index;
          uint8_t col_index;
          uint8_t pixels;
          if ((ch < 0x20U) || (ch > 0x7FU)) {
            ch = (uint8_t)'?';
          }
          row_index = (uint8_t)(ch - 0x20U);
          col_index = (uint8_t)(dx / scale);
          pixels = (uint8_t)ASCII[row_index][col_index];
          if (((pixels >> src_row) & 1U) != 0U) {
            color = fg;
          }
        }
        SPI_ControllerTx_16bit(color);
      }
      i++;
    }
  }
}

static void lcd_show_start_prompt(void)
{
  static const char line1[] = "PRESS BUTTON";
  static const char line2[] = "TO START";
  const uint8_t scale = 2U;
  const uint8_t glyph_step = 11U;
  const uint8_t space_step = 16U;

  lcd_init_once();
  LCD_setScreen(BLACK);

  lcd_draw_text_line_scaled_fast(3U, 18U, line1, scale, glyph_step, space_step, WHITE, BLACK);
  lcd_draw_text_line_scaled_fast(3U, 46U, line2, scale, glyph_step, space_step, WHITE, BLACK);
}
#endif







// static void v_LCD_Test_Task(void *pvParameters)
// {
//   static const uint16_t colors[] = {
//     BLACK,
//     RED,
//     GREEN,
//     BLUE,
//     WHITE
//   };
//   static const char *color_names[] = {
//     "black",
//     "red",
//     "green",
//     "blue",
//     "white"
//   };
//   uint32_t color_index = 0U;

//   (void)pvParameters;

//   osDelay(500);
//   printf("LCD: init start\r\n");
//   lcd_init_once();

//   while (1) {
//     printf("LCD: fill %s\r\n", color_names[color_index]);
//     LCD_setScreen(colors[color_index]);
//     color_index = (color_index + 1U) % (sizeof(colors) / sizeof(colors[0]));
//     osDelay(800);
//   }
// }

// Main application initialization and task creation.
void app_init(void *unused)
{
  UNUSED_PARAMETER(unused);

#if APP_BUTTON_TEST_ONLY
#if APP_LCD_ENABLED
  lcd_init_once();
#endif
  i2c_dual_imu_tasks_start();
#if APP_LCD_ENABLED
  lcd_show_start_prompt();
#endif
  button_init_instances();
  
  printf("APP: button test only mode\r\n");
  osThreadNew(v_BUTTON_Task, NULL, &(osThreadAttr_t){
      .name = "ButtonTask",
      .stack_size = 1024,
      .priority = osPriorityAboveNormal
  });

  return;
#endif

#if APP_IMU_DEBUG_ONLY
  printf("APP: IMU debug only mode\r\n");
  i2c_dual_imu_tasks_start();
  return;
#endif

#if APP_HAPTIC_DEBUG_ONLY
  printf("APP: haptic debug only mode\r\n");
#if APP_HAPTIC_DEBUG_WITH_LCD && APP_LCD_ENABLED
  lcd_init_once();
  lcd_show_start_prompt();
  printf("APP: haptic debug with LCD enabled\r\n");
#else
  (void)sl_gpio_driver_init();
#endif
#if APP_HAPTIC_DEBUG_WITH_BUTTON
  button_init_instances();
  printf("APP: haptic debug with button enabled\r\n");
  osThreadNew(v_BUTTON_Task, NULL, &(osThreadAttr_t){
      .name = "ButtonTask",
      .stack_size = 1024,
      .priority = osPriorityNormal
  });
#endif
  drv2605l_haptic_task_start();
  return;
#endif

#if APP_LCD_ENABLED
  lcd_init_once();
#elif APP_GPIO_INIT_WITHOUT_LCD
  (void)sl_gpio_driver_init();
#endif
  // lcd_init() triggers GPIO driver init; re-register button interrupts afterward.
  button_init_instances();
#if APP_LCD_ENABLED
  printf("LCD: show start prompt\r\n");
  lcd_show_start_prompt();
#else
  printf("LCD: disabled by APP_LCD_ENABLED=0\r\n");
#endif

#if ST7735_GPIO_TEST_ONLY
#if APP_LCD_ENABLED
  lcd_init_once();
#elif APP_GPIO_INIT_WITHOUT_LCD
  (void)sl_gpio_driver_init();
#endif
  // lcd_init() triggers GPIO driver init; re-register button interrupts afterward.
  button_init_instances();
#if APP_LCD_ENABLED
  printf("LCD: show start prompt\r\n");
  lcd_show_start_prompt();
  
  osThreadNew(v_LCD_Notify_Task, NULL, &(osThreadAttr_t){
      .name = "LCDNotifyTask",
      .stack_size = 1024,
      .priority = osPriorityNormal
  });
#else
  printf("LCD: disabled by APP_LCD_ENABLED=0\r\n");
#endif
  // osThreadNew(v_LCD_Test_Task, NULL, &(osThreadAttr_t){
  //     .name = "LCDTestTask",
  //     .stack_size = 1024,
  //     .priority = osPriorityNormal
  // });
#else
#if APP_IMU_ENABLED
  i2c_dual_imu_tasks_start();
#else
  printf("IMU: disabled by APP_IMU_ENABLED=0\r\n");
#endif
  drv2605l_haptic_task_start();
  // I2C/LCD/haptic init can touch shared GPIO driver state; refresh button IRQ last.
  button_init_instances();

  osThreadNew((osThreadFunc_t)application_start, NULL, &thread_attributes);

  osThreadNew(v_BUTTON_Task, NULL, &(osThreadAttr_t){
        .name = "ButtonTask",
        .stack_size = 1024, 
        .priority = osPriorityNormal
    });
 #if APP_LCD_ENABLED
  osThreadNew(v_LCD_Notify_Task, NULL, &(osThreadAttr_t){
        .name = "LCDNotifyTask",
        .stack_size = 1024,
        .priority = osPriorityNormal
    });
#endif
#if APP_IMU_ENABLED
  osThreadNew(v_IMU_MQTT_Task, NULL, &(osThreadAttr_t){
        .name = "IMUMQTTTask",
        .stack_size = 3072,
        .priority = osPriorityAboveNormal
    });
#endif

    // osThreadNew(v_LED_Task, NULL, &(osThreadAttr_t){
    //     .name = "LedTask",
    //     .stack_size = 1024,
    //     .priority = osPriorityNormal
    // });
    // osThreadNew(v_LCD_Test_Task, NULL, &(osThreadAttr_t){
    //     .name = "LCDTestTask",
    //     .stack_size = 1024,
    //     .priority = osPriorityNormal
    // });
#endif

}

#if LOAD_CERTIFICATE
// Load TLS CA certificates into flash memory when HTTPS is used.
sl_status_t clear_and_load_certificates_in_flash(void)
{
  sl_status_t status;
  void *cert           = NULL;
  uint32_t cert_length = 0;

#ifdef AWS_ENABLE
  cert        = (void *)aws_starfield_ca;
  cert_length = (sizeof(aws_starfield_ca) - 1);
#elif AZURE_ENABLE
  cert        = (void *)silabs_dgcert_ca;
  cert_length = (sizeof(silabs_dgcert_ca) - 1);
#else
  cert        = (uint8_t *)cacert;
  cert_length = (sizeof(cacert) - 1);
#endif

  //! Load SSL CA certificate
  status = sl_net_set_credential(SL_NET_TLS_SERVER_CREDENTIAL_ID(CERTIFICATE_INDEX),
                                 SL_NET_SIGNING_CERTIFICATE,
                                 cert,
                                 cert_length);
  if (status != SL_STATUS_OK) {
    printf("\r\nLoading TLS CA certificate in to FLASH Failed, Error Code : 0x%lX\r\n", status);
  } else {
    printf("\r\nLoad TLS CA certificate at index %d Success\r\n", CERTIFICATE_INDEX);
  }

  return status;
}
#endif

// Callback for Wi-Fi join events, used to detect connection failures.
sl_status_t join_callback_handler(sl_wifi_event_t event, char *result, uint32_t result_length, void *arg)
{
  UNUSED_PARAMETER(result);
  UNUSED_PARAMETER(arg);
  app_state = WLAN_UNCONNECTED_STATE;

  printf("\r\nIn Join CB\r\n");

  if (SL_WIFI_CHECK_IF_EVENT_FAILED(event)) {
    printf("F: Initiating rejoin %lu bytes payload\n", result_length);
    return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

void application_start(const void *unused)
{
  UNUSED_PARAMETER(unused);
  sl_status_t status = SL_STATUS_OK;
  uint16_t flags     = FLAGS;

  if (CERTIFICATE_INDEX == 1) {
    flags |= SL_SI91X_HTTPS_CERTIFICATE_INDEX_1;
  } else if (CERTIFICATE_INDEX == 2) {
    flags |= SL_SI91X_HTTPS_CERTIFICATE_INDEX_2;
  }


#if defined(AWS_ENABLE) || defined(AZURE_ENABLE)
#endif

  while (1) {
    switch (app_state) {
      case WLAN_INITIAL_STATE: {
        //! Client initialization
        status = sl_net_init(SL_NET_WIFI_CLIENT_INTERFACE, &station_init_configuration, NULL, NULL);
        if (status != SL_STATUS_OK) {
          printf("\r\nWi-Fi Client initialization failed , Error Code: 0x%lX\r\n", status);
          return;
        }
        printf("\r\nWi-Fi Client initialization success\r\n");
        

        
        printf("\r\nWi-Fi Client initialization success\r\n");

        sl_mac_address_t mac = { 0 };
        sl_status_t mac_status = sl_wifi_get_mac_address(SL_WIFI_CLIENT_INTERFACE, &mac);

        if (mac_status == SL_STATUS_OK) {
            printf("Device MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                   mac.octet[0], mac.octet[1], mac.octet[2], 
                   mac.octet[3], mac.octet[4], mac.octet[5]);
        } 





        //! Load certificate
#if LOAD_CERTIFICATE
        if (FLAGS & HTTPS_SUPPORT) {
          status = clear_and_load_certificates_in_flash();
          if (status != SL_STATUS_OK) {
            printf("\r\nUnexpected error while loading certificate: 0x%lX\r\n", status);
            return;
          }
        }
#endif
        app_state = WLAN_UNCONNECTED_STATE;
      } break;
      case WLAN_UNCONNECTED_STATE: {
        sl_wifi_set_join_callback(join_callback_handler, NULL);
        //! Bring up client interface
        status = sl_net_up(SL_NET_WIFI_CLIENT_INTERFACE, SL_NET_DEFAULT_WIFI_CLIENT_PROFILE_ID);
        if (status != SL_STATUS_OK) {
          printf("\r\nFailed to bring Wi-Fi client interface up: 0x%lX\r\n", status);
          app_state = WLAN_UNCONNECTED_STATE;
          break;
        }
        printf("\r\nWi-Fi Client interface up\r\n");
    
        sl_net_ip_address_t local_ip = { 0 };

        sl_status_t ip_status = sl_net_get_ip_address(SL_NET_WIFI_CLIENT_INTERFACE, 
                                                      &local_ip, 
                                                      5000); // 5000ms timeout

        if (ip_status == SL_STATUS_OK) {
            sl_ipv4_address_t my_ip = local_ip.v4.ip_address; 
            
            printf("DEBUG: My Local IP is: %u.%u.%u.%u\n",
                  (uint8_t)(my_ip.value & 0xFF), 
                  (uint8_t)((my_ip.value >> 8) & 0xFF),
                  (uint8_t)((my_ip.value >> 16) & 0xFF), 
                  (uint8_t)((my_ip.value >> 24) & 0xFF));
        } else {
            printf("DEBUG: Failed to get local IP, Error: 0x%lX\n", ip_status);
        }
        
        app_state = DONE;

        status = mqtt_example();
        if (ota_update_requested != 0U) {
          ota_update_requested = 0U;
          status               = run_http_otaf_update(flags);
          if (status != SL_STATUS_OK) {
            printf("\r\nOTA update failed: 0x%lX\r\n", status);
            return;
          }
        } else if (status != SL_STATUS_OK) {
          printf("\r\nMQTT example failed: 0x%lX\r\n", status);
        }

      } break;
      case CALIBRATION : {
        osDelay(osWaitForever);
      } break;
      case DATA_COLLECTION : {
        osDelay(osWaitForever);
      } break;
      case MODEL_ANALYTICS : {
        osDelay(osWaitForever);
      } break;
      case ANALYZED_DATA_PUBLISHING : {
        osDelay(osWaitForever);
      } break;

      default :{
        osDelay(osWaitForever);
        break;
      }
    }
  }
}

// Perform HTTP OTA firmware update and handle DNS/resolution as needed.
static sl_status_t run_http_otaf_update(uint16_t flags)
{
  sl_status_t status = SL_STATUS_OK;
  char server_ip[16];

#if (FW_UPDATE_TYPE == TA_FW_UPDATE)
  sl_wifi_firmware_version_t version = { 0 };

  status = sl_wifi_get_firmware_version(&version);
  if (status == SL_STATUS_OK) {
    print_firmware_version(&version);
  } else {
    printf("\r\nFailed to get firmware version before OTA: 0x%lX\r\n", status);
  }
#endif

#if defined(AWS_ENABLE) || defined(AZURE_ENABLE)
  sl_ip_address_t dns_query_rsp = { 0 };
  uint32_t server_address;
  int32_t dns_retry_count = MAX_DNS_RETRY_COUNT;
#endif

  sl_wifi_set_callback(SL_WIFI_HTTP_OTA_FW_UPDATE_EVENTS,
                       (sl_wifi_callback_function_t)&http_fw_update_response_handler,
                       NULL);

#if defined(AWS_ENABLE) || defined(AZURE_ENABLE)
  do {
    status = sl_net_dns_resolve_hostname((const char *)hostname, DNS_TIMEOUT, SL_NET_DNS_TYPE_IPV4, &dns_query_rsp);
    dns_retry_count--;
  } while ((dns_retry_count != 0) && (status != SL_STATUS_OK));

  if (status != SL_STATUS_OK) {
    printf("\r\nUnexpected error while resolving dns, Error 0x%lX\r\n", status);
    return status;
  }
  printf("\r\nResolving dns Success\r\n");

  server_address = dns_query_rsp.ip.v4.value;
  sprintf((char *)server_ip,
          "%ld.%ld.%ld.%ld",
          server_address & 0x000000ff,
          (server_address & 0x0000ff00) >> 8,
          (server_address & 0x00ff0000) >> 16,
          (server_address & 0xff000000) >> 24);
#ifdef AWS_ENABLE
  printf("\nResolved AWS S3 Bucket IP address = %s\n", server_ip);
#elif AZURE_ENABLE
  printf("\nResolved AZURE Blob Storage IP address = %s\n", server_ip);
#endif
#else
  strcpy(server_ip, HTTP_SERVER_IP_ADDRESS);
  printf("\r\n%s IP Address : %s\r\n", SERVER_NAME, HTTP_HOSTNAME);
#endif

  printf("\r\nFirmware download from %s is in progress...\r\n", SERVER_NAME);

  sl_si91x_http_otaf_params_t http_params = { 0 };

  http_params.flags           = (uint16_t)flags;
  http_params.ip_address      = (uint8_t *)server_ip;
  http_params.port            = (uint16_t)HTTP_PORT;
  http_params.resource        = (uint8_t *)HTTP_URL;
  http_params.host_name       = (uint8_t *)hostname;
  http_params.extended_header = (uint8_t *)HTTP_EXTENDED_HEADER;
  http_params.user_name       = (uint8_t *)USERNAME;
  http_params.password        = (uint8_t *)PASSWORD;

  printf("\r\nOTAF request flags=0x%X host=%s resource=%s port=%u ip=%s\r\n",
         http_params.flags,
         http_params.host_name,
         http_params.resource,
         http_params.port,
         http_params.ip_address);

  status = sl_si91x_http_otaf_v2(&http_params);

  if (status != SL_STATUS_OK) {
    printf("\r\nFirmware update status = 0x%lX\r\n", status);
    return status;
  }

  printf("\r\nCompleted firmware download using %s\r\n", SERVER_NAME);
  printf("\r\nUpdating the firmware...\r\n");

#if (FW_UPDATE_TYPE == TA_FW_UPDATE)
  status = sl_net_deinit(SL_NET_WIFI_CLIENT_INTERFACE);
  if (status != SL_STATUS_OK) {
    printf("\r\nError while wifi deinit: 0x%lX \r\n", status);
    return status;
  }
  printf("\r\nWi-Fi Deinit is successful\r\n");

#if SL_NCP_UART_INTERFACE
  printf("Waiting for firmware upgrade to complete\n");
  osDelay(40000);
  printf("Waiting Done\n");
#endif

  status = sl_net_init(SL_NET_WIFI_CLIENT_INTERFACE, &station_init_configuration, NULL, NULL);
  if (status != SL_STATUS_OK) {
    printf("Failed to start Wi-Fi client interface: 0x%lX\r\n", status);
    return status;
  }
  printf("\r\nWi-Fi Init success\r\n");

  status = sl_wifi_get_firmware_version(&version);
  if (status == SL_STATUS_OK) {
    print_firmware_version(&version);
  } else {
    printf("\r\nFailed to get firmware version after OTA: 0x%lX\r\n", status);
  }
#else
  printf("\r\nSoC Soft Reset initiated!\r\n");
  sl_si91x_soc_nvic_reset();
#endif

  return SL_STATUS_OK;
}

static sl_status_t http_fw_update_response_handler(sl_wifi_event_t event,
                                                   uint16_t *data,
                                                   uint32_t data_length,
                                                   void *arg)
{
  UNUSED_PARAMETER(data);
  UNUSED_PARAMETER(data_length);
  UNUSED_PARAMETER(arg);
  if (SL_WIFI_CHECK_IF_EVENT_FAILED(event)) {
    response = false;
    return SL_STATUS_FAIL;
  }
  response = true;
  return SL_STATUS_OK;
}



/******************************************************
 *             MQTT Function Definitions
 ******************************************************/




// Clean up MQTT client resources and reset state.
void mqtt_client_cleanup()
{
  SL_CLEANUP_MALLOC(client_credentails);
  mqtt_connected = 0U;
  imu_mqtt_publish_busy = 0U;
  is_execution_completed = 1;
}

void mqtt_client_message_handler(void *client, sl_mqtt_client_message_t *message, void *context)
{
  UNUSED_PARAMETER(context);
  //sl_status_t status;
  printf("Message Received on Topic: ");

  print_char_buffer((char *)message->topic, message->topic_length);
  print_char_buffer((char *)message->content, message->content_length);
  
  if ((message->topic_length == strlen(TOPIC_TO_BE_SUBSCRIBED))
      && (strncmp((char *)message->topic, TOPIC_TO_BE_SUBSCRIBED, message->topic_length) == 0)
      && (message->content_length == strlen(OTA_TRIGGER_MESSAGE))
      && (strncmp((char *)message->content, OTA_TRIGGER_MESSAGE, message->content_length) == 0)) {
    ota_update_requested = 1U;
    printf("OTA trigger received from MQTT, disconnecting broker before OTA\r\n");
    sl_mqtt_client_disconnect(client, 0);
    return;
  }

  if(strncmp((char *)message->content, "ON", message->content_length) == 0){
    led_flag = 1;
  } else if(strncmp((char *)message->content, "OFF", message->content_length) == 0){
    led_flag = 0;
  }
  
  printf("led_flag value: %d\r\n", led_flag);
  // Unsubscribing to already subscribed topic.
  // status = sl_mqtt_client_unsubscribe(client,
  //                                     (uint8_t *)TOPIC_TO_BE_SUBSCRIBED,
  //                                     strlen(TOPIC_TO_BE_SUBSCRIBED),
  //                                     0,
  //                                     TOPIC_TO_BE_SUBSCRIBED);
  // if (status != SL_STATUS_IN_PROGRESS) {
  //   printf("Failed to unsubscribe : 0x%lx\r\n", status);

  //   mqtt_client_cleanup();
  //   return;
  // }
}

// Print a raw character buffer to the console.
void print_char_buffer(char *buffer, uint32_t buffer_length)
{
  for (uint32_t index = 0; index < buffer_length; index++) {
    printf("%c", buffer[index]);
  }

  printf("\r\n");
}

void mqtt_client_error_event_handler(void *client, sl_mqtt_client_error_status_t *error)
{
  UNUSED_PARAMETER(client);
  printf("Terminating program, Error: %d\r\n", *error);
  mqtt_client_cleanup();
}

void mqtt_client_event_handler(void *client, sl_mqtt_client_event_t event, void *event_data, void *context)
{
  switch (event) {
    case SL_MQTT_CLIENT_CONNECTED_EVENT: {
      sl_status_t status;
      mqtt_connected = 1U;

      status = sl_mqtt_client_subscribe(client,
                                        (uint8_t *)TOPIC_TO_BE_SUBSCRIBED,
                                        strlen(TOPIC_TO_BE_SUBSCRIBED),
                                        QOS_OF_SUBSCRIPTION,
                                        0,
                                        mqtt_client_message_handler,
                                        TOPIC_TO_BE_SUBSCRIBED);
      if (status != SL_STATUS_IN_PROGRESS) {
        printf("Failed to subscribe : 0x%lx\r\n", status);

        mqtt_client_cleanup();
        return;
      }

      status = sl_mqtt_client_publish(client, &message_to_be_published, 0, &message_to_be_published);
      if (status != SL_STATUS_IN_PROGRESS) {
        printf("Failed to publish message: 0x%lx\r\n", status);

        mqtt_client_cleanup();
        return;
      }

      break;
    }

    case SL_MQTT_CLIENT_MESSAGE_PUBLISHED_EVENT: {
      (void)context;
      imu_mqtt_publish_busy = 0U;
      break;
    }

    case SL_MQTT_CLIENT_SUBSCRIBED_EVENT: {
      char *subscribed_topic = (char *)context;

      printf("Subscribed to Topic: %s\r\n", subscribed_topic);
      break;
    }

    case SL_MQTT_CLIENT_UNSUBSCRIBED_EVENT: {
      char *unsubscribed_topic = (char *)context;

      printf("Unsubscribed from topic: %s\r\n", unsubscribed_topic);

      sl_mqtt_client_disconnect(client, 0);
      break;
    }

    case SL_MQTT_CLIENT_DISCONNECTED_EVENT: {
      mqtt_connected = 0U;
      imu_mqtt_publish_busy = 0U;
      printf("Disconnected from MQTT broker\r\n");

      mqtt_client_cleanup();
      break;
    }

    case SL_MQTT_CLIENT_ERROR_EVENT: {
      mqtt_connected = 0U;
      imu_mqtt_publish_busy = 0U;
      mqtt_client_error_event_handler(client, (sl_mqtt_client_error_status_t *)event_data);
      break;
    }
    default:
      break;
  }
}

// Run the MQTT client example flow, including broker connect and message handling.
sl_status_t mqtt_example()
{
  sl_status_t status;
  is_execution_completed = 0U;

  if (ENCRYPT_CONNECTION) {
    // Load SSL CA certificate
    status = sl_net_set_credential(SL_NET_TLS_SERVER_CREDENTIAL_ID(CERTIFICATE_INDEX),
                                   SL_NET_SIGNING_CERTIFICATE,
                                   cacert,
                                   sizeof(cacert) - 1);
    if (status != SL_STATUS_OK) {
      printf("\r\nLoading TLS CA certificate in to FLASH Failed, Error Code : 0x%lX\r\n", status);
      return status;
    }
    printf("\r\nLoad TLS CA certificate at index %d Success\r\n", CERTIFICATE_INDEX);
  }

  if (SEND_CREDENTIALS) {
    uint16_t username_length, password_length;

    username_length = strlen(USERNAME);
    password_length = strlen(PASSWORD);

    uint32_t malloc_size = sizeof(sl_mqtt_client_credentials_t) + username_length + password_length;

    client_credentails = malloc(malloc_size);
    if (client_credentails == NULL)
      return SL_STATUS_ALLOCATION_FAILED;
    memset(client_credentails, 0, malloc_size);
    client_credentails->username_length = username_length;
    client_credentails->password_length = password_length;

    memcpy(&client_credentails->data[0], USERNAME, username_length);
    memcpy(&client_credentails->data[username_length], PASSWORD, password_length);

    status = sl_net_set_credential(SL_NET_MQTT_CLIENT_CREDENTIAL_ID(0),
                                   SL_NET_MQTT_CLIENT_CREDENTIAL,
                                   client_credentails,
                                   malloc_size);

    if (status != SL_STATUS_OK) {
      mqtt_client_cleanup();
      printf("Failed to set credentials: 0x%lx\r\n ", status);

      return status;
    }
    printf("Set credentials Success \r\n ");

    free(client_credentails);
    mqtt_client_configuration.credential_id = SL_NET_MQTT_CLIENT_CREDENTIAL_ID(0);
  }

  status = sl_mqtt_client_init(&client, mqtt_client_event_handler);
  if (status != SL_STATUS_OK) {
    printf("Failed to init mqtt client: 0x%lx\r\n", status);

    mqtt_client_cleanup();
    return status;
  }
  printf("Init mqtt client Success \r\n");

#ifdef SLI_SI91X_ENABLE_IPV6
  unsigned char hex_addr[SL_IPV6_ADDRESS_LENGTH] = { 0 };
  status                                         = sl_inet_pton6(MQTT_BROKER_IP,
                         MQTT_BROKER_IP + strlen(MQTT_BROKER_IP),
                         hex_addr,
                         (unsigned int *)mqtt_broker_configuration.ip.ip.v6.value);
  if (status != 0x1) {
    printf("\r\nIPv6 conversion failed.\r\n");
    mqtt_client_cleanup();
    return status;
  }
  mqtt_broker_configuration.ip.type = SL_IPV6;
#else
  status = sl_net_inet_addr(MQTT_BROKER_IP, &mqtt_broker_configuration.ip.ip.v4.value);
  if (status != SL_STATUS_OK) {
    printf("Failed to convert IP address \r\n");

    mqtt_client_cleanup();
    return status;
  }
  mqtt_broker_configuration.ip.type = SL_IPV4;
#endif

  status =
    sl_mqtt_client_connect(&client, &mqtt_broker_configuration, &last_will_message, &mqtt_client_configuration, 0);
  if (status != SL_STATUS_IN_PROGRESS) {
    printf("Failed to connect to mqtt broker: 0x%lx\r\n", status);

    mqtt_client_cleanup();
    return status;
  }
  printf("Connect to mqtt broker Success \r\n");

  while (!is_execution_completed) {
    osThreadYield();
  }

  printf("Example execution completed \r\n");

  return SL_STATUS_OK;
}
