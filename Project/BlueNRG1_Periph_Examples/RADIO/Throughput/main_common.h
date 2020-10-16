/**
  ******************************************************************************
  * @file    main_common.h 
  * @author  RF Application Team
* @version V1.0.1
* @date    April-2018
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_COMMON_H
#define MAIN_COMMON_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/
#define CALIBRATION_INTERVAL_CONF   1000

#if LS_SOURCE==LS_SOURCE_INTERNAL_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#else

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x888888DF)
#define FREQUENCY_CHANNEL       (uint8_t)(24)    // RF channel 22
#define HS_STARTUP_TIME         (uint16_t)(1)   /* High Speed start up time min value */

#ifdef UNIDIRECTIONAL_TEST
#if HS_SPEED_XTAL==HS_SPEED_XTAL_16MHZ
#define TX_WAKEUP_TIME           (225)
#else
#define TX_WAKEUP_TIME           (200)
#endif
#else
#define TX_WAKEUP_TIME           (250)
#endif
#if HS_SPEED_XTAL==HS_SPEED_XTAL_16MHZ
#define RX_WAKEUP_TIME           (235)
#else
#define RX_WAKEUP_TIME           (210)
#endif
#define RX_TIMEOUT            100000      /* 100 ms */
#define RX_TIMEOUT_ACK           150      /* 150 us */

/* Exported functions ------------------------------------------------------- */


#endif /* MAIN_COMMON_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
