/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct __attribute__((packed)) {
    int16_t x;       /* -32767 to 32767 (Twist Left / Twist Right) */
    int16_t y;       /* -32767 to 32767 (Top / Down) */
    int16_t z;       /* -32767 to 32767 (Right / Left) */
    uint8_t buttons; /* Bit 0=PB12, Bit 1=PB13, Bit 2=PB14, Bit 3=PB15, Bit 4=PA8, Bit 5=PA9 */
} HID_GamepadReport_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JOY_X_Pin             GPIO_PIN_1
#define JOY_X_GPIO_Port       GPIOA
#define JOY_Y_Pin             GPIO_PIN_2
#define JOY_Y_GPIO_Port       GPIOA
#define JOY_Z_Pin             GPIO_PIN_3
#define JOY_Z_GPIO_Port       GPIOA

#define BTN_1_Pin             GPIO_PIN_12
#define BTN_1_GPIO_Port       GPIOB
#define BTN_2_Pin             GPIO_PIN_13
#define BTN_2_GPIO_Port       GPIOB
#define BTN_3_Pin             GPIO_PIN_14
#define BTN_3_GPIO_Port       GPIOB
#define BTN_4_Pin             GPIO_PIN_15
#define BTN_4_GPIO_Port       GPIOB

#define BTN_5_Pin             GPIO_PIN_8
#define BTN_5_GPIO_Port       GPIOA
#define BTN_6_Pin             GPIO_PIN_9
#define BTN_6_GPIO_Port       GPIOA

#define LED_STATUS_Pin        GPIO_PIN_13
#define LED_STATUS_GPIO_Port  GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
