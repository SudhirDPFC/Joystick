/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programmable USB HID Gamepad Controller
  *                   STM32F103C8T6 (Blue Pill) - 48MHz Clock, 5-Layer ADC Filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_hid.h"
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ═══════════════════════════════════════════════════════════════════════════
 *  BUTTON TIMERS & MASKS (EXTI Interrupts on PB12..PB15, PA8, PA9)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define DEBOUNCE_MS              200

volatile uint32_t lastPB12 = 0;
volatile uint32_t lastPB13 = 0;
volatile uint32_t lastPB14 = 0;
volatile uint32_t lastPB15 = 0;
volatile uint32_t lastPA8  = 0;
volatile uint32_t lastPA9  = 0;

volatile uint8_t  button_state_mask = 0;
volatile uint8_t  immediate_dispatch_flag = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 *  JOYSTICK 5-LAYER FILTER PIPELINE PARAMETERS (from T-3)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define JOY_DEADZONE_ENTER      125    /* Deflection required to activate direction */
#define JOY_DEADZONE_EXIT        75    /* Deflection required to return to center */

#define DOMINANCE_MULT_NUM       27
#define DOMINANCE_MULT_DEN       20

#define JOY_SAMPLES              12    /* 12x oversampling (double-trimmed mean) */
#define JOY_REPEAT_MS           200    /* 200ms periodic heartbeat dispatch */
#define HOLD_GATE_ENTER_MS       60    /* 60ms hold confirmation */
#define HOLD_GATE_EXIT_MS        45    /* 45ms release confirmation */

#define ZONE_LOW                  0
#define ZONE_CENTER               1
#define ZONE_HIGH                 2

/* Resting centers auto-calibrated at boot */
int joyCenterX = 285;
int joyCenterY = 285;
int joyCenterZ = 285;

uint8_t confirmedZoneX = ZONE_CENTER;
uint8_t confirmedZoneY = ZONE_CENTER;
uint8_t confirmedZoneZ = ZONE_CENTER;

uint8_t candidateZoneX = ZONE_CENTER;
uint8_t candidateZoneY = ZONE_CENTER;
uint8_t candidateZoneZ = ZONE_CENTER;

uint32_t candidateStartTimeX = 0;
uint32_t candidateStartTimeY = 0;
uint32_t candidateStartTimeZ = 0;

uint32_t lastJoyDispatchTime = 0;

/* Layer 2: Fixed-point EMA filter */
int emaX = 285, emaY = 285, emaZ = 285;

/* Layer 3: 7-Sample circular median filter */
#define MEDIAN_SIZE 7
int medBufX[MEDIAN_SIZE];
int medBufY[MEDIAN_SIZE];
int medBufZ[MEDIAN_SIZE];
uint8_t medIdx = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void USB_SoftwareDisconnect(void);
static uint16_t Read_ADC_Channel(uint32_t channel);
static int robustReadADC(uint32_t channel);
static int getCircularMedian(int* buf, int val);
static void updateZoneGated(uint8_t newCandidate, uint8_t *candidate, uint8_t *confirmed, uint32_t *startTime, uint32_t now);
static void handleJoystick(HID_GamepadReport_t *pReport);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes Flash interface and Systick */
  HAL_Init();

  /* Configure the system clock (48MHz HSE PLL, 48MHz USB, 12MHz ADC) */
  SystemClock_Config();

  /* Force USB Host re-enumeration on Blue Pill (pulls D+ LOW for 25ms) */
  USB_SoftwareDisconnect();

  /* Initialize GPIO with EXTI Interrupts on buttons and LED */
  MX_GPIO_Init();

  /* Initialize ADC1 for analog joystick reads */
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Auto-Zero Joystick Center Calibration on boot (40 samples) */
  long sumX = 0, sumY = 0, sumZ = 0;
  const int CAL_SAMPLES = 40;
  for (int i = 0; i < CAL_SAMPLES; i++)
  {
    sumX += Read_ADC_Channel(ADC_CHANNEL_1);
    sumY += Read_ADC_Channel(ADC_CHANNEL_2);
    sumZ += Read_ADC_Channel(ADC_CHANNEL_3);
    HAL_Delay(3);
  }
  int avgX = (int)(sumX / CAL_SAMPLES);
  int avgY = (int)(sumY / CAL_SAMPLES);
  int avgZ = (int)(sumZ / CAL_SAMPLES);

  /* Sanity check: valid center range (200..380), otherwise fallback to 285 */
  if (avgX >= 200 && avgX <= 380) joyCenterX = avgX;
  if (avgY >= 200 && avgY <= 380) joyCenterY = avgY;
  if (avgZ >= 200 && avgZ <= 380) joyCenterZ = avgZ;

  emaX = joyCenterX; emaY = joyCenterY; emaZ = joyCenterZ;
  for (int i = 0; i < MEDIAN_SIZE; i++)
  {
    medBufX[i] = joyCenterX;
    medBufY[i] = joyCenterY;
    medBufZ[i] = joyCenterZ;
  }

  /* Initialize USB Device Stack as HID Gamepad */
  MX_USB_DEVICE_Init();

  HID_GamepadReport_t report = {0};

  /* Infinite loop */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 1. Button Release Polling (Active-LOW inputs: HIGH = Released) */
    if (button_state_mask != 0)
    {
      uint8_t old_mask = button_state_mask;
      if ((button_state_mask & 0x01) && HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET) button_state_mask &= ~0x01;
      if ((button_state_mask & 0x02) && HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_SET) button_state_mask &= ~0x02;
      if ((button_state_mask & 0x04) && HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_SET) button_state_mask &= ~0x04;
      if ((button_state_mask & 0x08) && HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) == GPIO_PIN_SET) button_state_mask &= ~0x08;
      if ((button_state_mask & 0x10) && HAL_GPIO_ReadPin(BTN_5_GPIO_Port, BTN_5_Pin) == GPIO_PIN_SET) button_state_mask &= ~0x10;
      if ((button_state_mask & 0x20) && HAL_GPIO_ReadPin(BTN_6_GPIO_Port, BTN_6_Pin) == GPIO_PIN_SET) button_state_mask &= ~0x20;

      if (button_state_mask != old_mask)
      {
        immediate_dispatch_flag = 1;
      }
    }

    /* 2. Process 5-Layer Joystick Filter Pipeline */
    handleJoystick(&report);

    /* 3. Send HID Report: Immediately on button press/release or every 200ms periodic heartbeat */
    if (immediate_dispatch_flag || (now - lastJoyDispatchTime >= JOY_REPEAT_MS))
    {
      lastJoyDispatchTime = now;
      immediate_dispatch_flag = 0;

      report.buttons = button_state_mask;

      if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
        /* Toggle LED to indicate activity */
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
      }
    }
  }
}

/**
  * @brief System Clock Configuration (48MHz HSE PLL)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Enable HSE 8MHz and PLL x6 -> 48MHz SYSCLK */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure bus dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure 48MHz USB Clock (/1) and 12MHz ADC Clock (/4) */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Software USB disconnect pulse on Blue Pill
  */
static void USB_SoftwareDisconnect(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Drive PA12 LOW to signal disconnect */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(25);

  /* Float PA12 as input */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_Delay(10);
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure PA1, PA2, PA3 as Analog Inputs */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = JOY_X_Pin | JOY_Y_Pin | JOY_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Default channel config */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief GPIO Initialization Function (Buttons with EXTI Falling Edge & LED)
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure Onboard Status LED (PC13, Active LOW) */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /* Configure Button Pins (PB12..PB15, PA8, PA9) with EXTI Falling Edge and Pull-Up */
  GPIO_InitStruct.Pin = BTN_1_Pin | BTN_2_Pin | BTN_3_Pin | BTN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BTN_5_Pin | BTN_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable EXTI Interrupts in NVIC */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief EXTI Falling Edge Callback for Buttons
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now = HAL_GetTick();

  switch (GPIO_Pin)
  {
    case BTN_1_Pin:
      if (now - lastPB12 > DEBOUNCE_MS && HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET)
      {
        lastPB12 = now;
        button_state_mask |= 0x01;
        immediate_dispatch_flag = 1;
      }
      break;

    case BTN_2_Pin:
      if (now - lastPB13 > DEBOUNCE_MS && HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET)
      {
        lastPB13 = now;
        button_state_mask |= 0x02;
        immediate_dispatch_flag = 1;
      }
      break;

    case BTN_3_Pin:
      if (now - lastPB14 > DEBOUNCE_MS && HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_RESET)
      {
        lastPB14 = now;
        button_state_mask |= 0x04;
        immediate_dispatch_flag = 1;
      }
      break;

    case BTN_4_Pin:
      if (now - lastPB15 > DEBOUNCE_MS && HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) == GPIO_PIN_RESET)
      {
        lastPB15 = now;
        button_state_mask |= 0x08;
        immediate_dispatch_flag = 1;
      }
      break;

    case BTN_5_Pin:
      if (now - lastPA8 > DEBOUNCE_MS && HAL_GPIO_ReadPin(BTN_5_GPIO_Port, BTN_5_Pin) == GPIO_PIN_RESET)
      {
        lastPA8 = now;
        button_state_mask |= 0x10;
        immediate_dispatch_flag = 1;
      }
      break;

    case BTN_6_Pin:
      if (now - lastPA9 > DEBOUNCE_MS && HAL_GPIO_ReadPin(BTN_6_GPIO_Port, BTN_6_Pin) == GPIO_PIN_RESET)
      {
        lastPA9 = now;
        button_state_mask |= 0x20;
        immediate_dispatch_flag = 1;
      }
      break;

    default:
      break;
  }
}

/**
  * @brief Read single ADC channel in 10-bit resolution (matches Arduino analogRead)
  */
static uint16_t Read_ADC_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    return (uint16_t)(raw >> 2); /* 10-bit range (0..1023) */
  }
  return 285;
}

/**
  * @brief 12-Sample Double-Trimmed Mean ADC Read (Layer 1)
  */
static int robustReadADC(uint32_t channel)
{
  int samples[JOY_SAMPLES];
  for (int i = 0; i < JOY_SAMPLES; i++)
  {
    samples[i] = Read_ADC_Channel(channel);
    for (volatile int d = 0; d < 200; d++) { __NOP(); }
  }

  /* Insertion sort */
  for (int i = 1; i < JOY_SAMPLES; i++)
  {
    int key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key)
    {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  /* Sum middle 8 samples (discard 2 lowest and 2 highest) */
  long sum = 0;
  for (int i = 2; i < JOY_SAMPLES - 2; i++)
  {
    sum += samples[i];
  }
  return (int)(sum / (JOY_SAMPLES - 4));
}

/**
  * @brief 7-Sample Circular Median Filter (Layer 3)
  */
static int getCircularMedian(int* buf, int val)
{
  buf[medIdx] = val;
  int a[MEDIAN_SIZE];
  for (int i = 0; i < MEDIAN_SIZE; i++) a[i] = buf[i];

  for (int i = 1; i < MEDIAN_SIZE; i++)
  {
    int key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key)
    {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
  return a[3];
}

/**
  * @brief Time-Gated Zone State Machine (Layer 5)
  */
static void updateZoneGated(uint8_t newCandidate, uint8_t *candidate, uint8_t *confirmed, uint32_t *startTime, uint32_t now)
{
  if (newCandidate != *candidate)
  {
    *candidate = newCandidate;
    *startTime = now;
  }
  else if (*candidate != *confirmed)
  {
    uint32_t requiredHold = (*candidate == ZONE_CENTER) ? HOLD_GATE_EXIT_MS : HOLD_GATE_ENTER_MS;
    if (now - *startTime >= requiredHold)
    {
      *confirmed = *candidate;
    }
  }
}

/**
  * @brief Complete 5-Layer Joystick Filter Pipeline
  */
static void handleJoystick(HID_GamepadReport_t *pReport)
{
  uint32_t now = HAL_GetTick();

  /* 1. Stage 1: 12-Sample Double-Trimmed Mean */
  int rawX = robustReadADC(ADC_CHANNEL_1);
  int rawY = robustReadADC(ADC_CHANNEL_2);
  int rawZ = robustReadADC(ADC_CHANNEL_3);

  /* 2. Stage 2: Fixed-Point EMA Filter (alpha = 0.25) */
  emaX = (emaX * 3 + rawX) / 4;
  emaY = (emaY * 3 + rawY) / 4;
  emaZ = (emaZ * 3 + rawZ) / 4;

  /* 3. Stage 3: 7-Sample Circular Median Filter */
  int filtX = getCircularMedian(medBufX, emaX);
  int filtY = getCircularMedian(medBufY, emaY);
  int filtZ = getCircularMedian(medBufZ, emaZ);
  medIdx = (medIdx + 1) % MEDIAN_SIZE;

  /* 4. Stage 4: Deviations from Auto-Zeroed Center */
  int devX = filtX - joyCenterX;
  int devY = filtY - joyCenterY;
  int devZ = filtZ - joyCenterZ;

  int absDevX = abs(devX);
  int absDevY = abs(devY);
  int absDevZ = abs(devZ);

  /* 4b. 3D Dominant-Axis Crosstalk Evaluation */
  uint8_t newCandidateX = ZONE_CENTER;
  uint8_t newCandidateY = ZONE_CENTER;
  uint8_t newCandidateZ = ZONE_CENTER;

  int zIsDominant = (absDevZ >= JOY_DEADZONE_ENTER) &&
                    ((absDevZ * DOMINANCE_MULT_DEN) >= (absDevX * DOMINANCE_MULT_NUM)) &&
                    ((absDevZ * DOMINANCE_MULT_DEN) >= (absDevY * DOMINANCE_MULT_NUM));

  /* X Axis (TWIST LEFT / TWIST RIGHT) */
  if (!zIsDominant && absDevX >= JOY_DEADZONE_ENTER && (absDevX * DOMINANCE_MULT_DEN) >= (absDevY * DOMINANCE_MULT_NUM))
  {
    newCandidateX = (devX < 0) ? ZONE_LOW : ZONE_HIGH;
  }
  else if (confirmedZoneX != ZONE_CENTER && absDevX > JOY_DEADZONE_EXIT && (absDevX * DOMINANCE_MULT_DEN) >= (absDevY * 15))
  {
    newCandidateX = confirmedZoneX;
  }

  /* Y Axis (TOP / DOWN) */
  if (!zIsDominant && absDevY >= JOY_DEADZONE_ENTER && (absDevY * DOMINANCE_MULT_DEN) >= (absDevX * DOMINANCE_MULT_NUM))
  {
    newCandidateY = (devY < 0) ? ZONE_LOW : ZONE_HIGH;
  }
  else if (confirmedZoneY != ZONE_CENTER && absDevY > JOY_DEADZONE_EXIT && (absDevY * DOMINANCE_MULT_DEN) >= (absDevX * 15))
  {
    newCandidateY = confirmedZoneY;
  }

  /* Z Axis (RIGHT / LEFT) */
  if (absDevZ >= JOY_DEADZONE_ENTER && (absDevZ * DOMINANCE_MULT_DEN) >= (absDevX * 15) && (absDevZ * DOMINANCE_MULT_DEN) >= (absDevY * 15))
  {
    newCandidateZ = (devZ < 0) ? ZONE_LOW : ZONE_HIGH;
  }
  else if (confirmedZoneZ != ZONE_CENTER && absDevZ > JOY_DEADZONE_EXIT)
  {
    newCandidateZ = confirmedZoneZ;
  }

  /* 5. Stage 5: Time-Gated Confirmation */
  updateZoneGated(newCandidateX, &candidateZoneX, &confirmedZoneX, &candidateStartTimeX, now);
  updateZoneGated(newCandidateY, &candidateZoneY, &confirmedZoneY, &candidateStartTimeY, now);
  updateZoneGated(newCandidateZ, &candidateZoneZ, &confirmedZoneZ, &candidateStartTimeZ, now);

  /* Map to Signed 16-bit Axis Outputs */
  int16_t hidX = 0;
  int16_t hidY = 0;
  int16_t hidZ = 0;

  if      (confirmedZoneX == ZONE_LOW)  hidX = -32767;
  else if (confirmedZoneX == ZONE_HIGH) hidX =  32767;

  if      (confirmedZoneY == ZONE_LOW)  hidY = -32767;
  else if (confirmedZoneY == ZONE_HIGH) hidY =  32767;

  if      (confirmedZoneZ == ZONE_LOW)  hidZ = -32767;
  else if (confirmedZoneZ == ZONE_HIGH) hidZ =  32767;

  pReport->x = hidX;
  pReport->y = hidY;
  pReport->z = hidZ;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
