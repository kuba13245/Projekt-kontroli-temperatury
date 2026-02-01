/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "heater.h"
#include "pid_controller.h"
#include "fan.h"
#include "encoder_logic.h"
#include "pid_cooling.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_TIME_MS 100  // Czas cyklu pętli sterowania (10Hz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// --- Urządzenia i Struktury ---
BMP280_HandleTypedef bmp280;
Heater_TypeDef myHeater;
Fan_TypeDef myFan;
UI_StateTypeDef myEncoder;

// --- Kontrolery PID ---
PID_TypeDef hPID;          // Grzanie
PID_Cooling_TypeDef hPID_Cool; // Chłodzenie

// Nastawy PID Grzania
float Kp = 1500.0f;
float Ki = 1.2f;
float Kd = 2850.0f;

// Nastawy PID Chłodzenia
float Kp_c = 600.0f;
float Ki_c = 0.3f;
float Kd_c = 2500.0f;

// --- Zmienne Procesowe ---
float pressure, actual_temp, humidity;
float heating_power;
float sterowanie_fan;
float temp_min = 25.0f;
float temp_max = 38.0f;

int fan_percent;
int heater_percent;

// --- Komunikacja UART ---
uint16_t size;
uint8_t Data[256];      // Bufor nadawczy
uint8_t rx_byte;        // Znak odbierany w przerwaniu
uint8_t rx_buffer[32];  // Bufor komendy
uint8_t rx_index = 0;
volatile uint8_t line_received = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Calculate_CRC16(uint8_t *data, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // --- Inicjalizacja BMP280 ---
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;

  while (!bmp280_init(&bmp280, &bmp280.params)) {
      size = sprintf((char *)Data, "BMP280 initialization failed\n");
      HAL_UART_Transmit(&huart3, Data, size, 1000);
      HAL_Delay(2000);
  }

  // --- Inicjalizacja Podzespołów Wykonawczych ---
  Heater_Init(&myHeater, &htim2, TIM_CHANNEL_1, 5000.0f);
  Fan_Init(&myFan, &htim2, TIM_CHANNEL_4, 5000.0f);

  // --- Inicjalizacja Kontrolerów ---
  PID_Init(&hPID, 2200.0f);
  PID_Cooling_Init(&hPID_Cool, 5000.0f);

  // --- Inicjalizacja Interfejsu ---
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  EncoderInit(&myEncoder, &htim1, Encoder_btn_GPIO_Port, Encoder_btn_Pin, 25.0f, 0.10f, temp_min, temp_max);

  lcd_init();
  lcd_clear();

  // Statyczne napisy na LCD
  lcd_put_cur(0, 0);
  lcd_send_string("T:");
  lcd_put_cur(1, 0);
  lcd_send_string("S:");

  char lcd_buffer[16];

  // Start nasłuchu UART
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  uint32_t loop_timer = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // =================================================================
      // OBSŁUGA KOMEND UART
      // =================================================================
      if (line_received) {
          // Kopia bufora i zamiana na DUŻE LITERY
          char clean_cmd[32];
          strcpy(clean_cmd, (char*)rx_buffer);

          for(int i=0; i<strlen(clean_cmd); i++) {
              if(clean_cmd[i] >= 'a' && clean_cmd[i] <= 'z') {
                  clean_cmd[i] -= 32;
              }
          }

          // Szukamy separatora '*' (CRC)
          char *separator = strrchr(clean_cmd, '*');
          uint8_t command_valid = 0;

          if (separator != NULL) {
              // Rozdzielenie komendy i CRC
              *separator = '\0';
              char *crc_part = separator + 1;

              uint16_t calc_crc = Calculate_CRC16((uint8_t*)clean_cmd, strlen(clean_cmd));
              uint16_t recv_crc = (uint16_t)strtol(crc_part, NULL, 16);

              if (calc_crc == recv_crc) {
                  command_valid = 1;
              }
          }

          // Wykonanie komendy
          if (command_valid) {
              if (clean_cmd[0] == 'T') {
                  int raw_val = atoi(&clean_cmd[1]);
                  float new_set_temp = (float)raw_val / 10.0f;

                  if (new_set_temp >= temp_min && new_set_temp <= temp_max) {
                      myEncoder.set_temp = new_set_temp;
                      // Przeliczenie licznika enkodera
                      uint32_t new_counter = (uint32_t)(new_set_temp / 0.10f * 4.0f);
                      __HAL_TIM_SET_COUNTER(&htim1, new_counter);

                      size = sprintf((char *)Data, "CMD OK: Set %.2f C\r\n", new_set_temp);
                  } else {
                      size = sprintf((char *)Data, "CMD ERROR: Range %.1f-%.1f\r\n", temp_min, temp_max);
                  }
              }
              else if (strcmp(clean_cmd, "ON") == 0) {
                  myEncoder.isHeatingEnabled = 1;
                  size = sprintf((char *)Data, "CMD OK: ON\r\n");
              }
              else if (strcmp(clean_cmd, "OFF") == 0) {
                  myEncoder.isHeatingEnabled = 0;
                  size = sprintf((char *)Data, "CMD OK: OFF\r\n");
              }
              else {
                  size = sprintf((char *)Data, "UNKNOWN CMD: [%s]\r\n", clean_cmd);
              }
              HAL_UART_Transmit(&huart3, Data, size, 100);
          }

          // Reset bufora odbiorczego
          line_received = 0;
          memset(rx_buffer, 0, sizeof(rx_buffer));
      }

      // =================================================================
      // PĘTLA STEROWANIA
      // =================================================================
      if (HAL_GetTick() - loop_timer >= LOOP_TIME_MS)
      {
          loop_timer = HAL_GetTick();

          // Odczyt BMP280
          if (!bmp280_read_float(&bmp280, &actual_temp, &pressure, &humidity)) {

          }

          // Obsługa Enkodera (zmiany ręczne)
          EncoderUpdate(&myEncoder);

          // Logika Sterowania (PID)
          if (myEncoder.isHeatingEnabled) {


              // PID Grzania
              heating_power = PID_Calculate(&hPID, myEncoder.set_temp, actual_temp, Kp, Ki, Kd);
              Heater_SetPower(&myHeater, heating_power);

              // PID Chłodzenia
              sterowanie_fan = PID_Cooling_Calculate(&hPID_Cool, myEncoder.set_temp, actual_temp, Kp_c, Ki_c, Kd_c);

              // Minimalne obroty wentylatora (kick-start)
              if (sterowanie_fan > 0.0f && sterowanie_fan < 1000.0f) {
                  sterowanie_fan = 1000.0f;
              } else if (sterowanie_fan == 0.0f) {
                  sterowanie_fan = 0.0f;
              }

              Fan_SetPower(&myFan, sterowanie_fan);
          }
          else {
              // System wyłączony
              hPID.prevError = 0;
              hPID.integral = 0;

              heating_power = 0;
              sterowanie_fan = 0;

              Heater_SetPower(&myHeater, 0.0f);
              Fan_SetPower(&myFan, 0.0f);
          }

          if (myEncoder.isEditMode) {
        	  sprintf(lcd_buffer, "<");
        	  lcd_put_cur(1, 9);
        	  lcd_send_string(lcd_buffer);
          }
          else {
              sprintf(lcd_buffer, " ");
              lcd_put_cur(1, 9);
              lcd_send_string(lcd_buffer);
          }

          // Skalowanie do wyświetlania
          fan_percent = (int)(sterowanie_fan / 50.0f);
          heater_percent = (int)(heating_power / 22.0f);

          // Wysyłanie danych UART (Format stały dla Simulink/Terminala)
          size = sprintf((char *)Data, "$%04d,%04d,%04d,%04d\r\n",
                         (int)(myEncoder.set_temp * 100),
                         (int)(actual_temp * 100),
                         (int)heater_percent,
                         (int)fan_percent);
          HAL_UART_Transmit(&huart3, Data, size, 50);

          // Aktualizacja LCD
          sprintf(lcd_buffer, "%.2f%cC F:%d%% ", actual_temp, 0xDF, fan_percent);
          lcd_put_cur(0, 2);
          lcd_send_string(lcd_buffer);

          sprintf(lcd_buffer, "%.2f%cC", myEncoder.set_temp, 0xDF);
          lcd_put_cur(1, 2);
          lcd_send_string(lcd_buffer);

          sprintf(lcd_buffer, "H:%d%%  ", heater_percent);
          lcd_put_cur(1, 10);
          lcd_send_string(lcd_buffer);
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : Encoder_btn_Pin */
  GPIO_InitStruct.Pin = Encoder_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder_btn_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                line_received = 1;
                rx_index = 0;
            }
        }
        else {
            if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

uint16_t Calculate_CRC16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0; // Start od 0 (dla wyniku B4B5)

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i]; // XOR bajtu danych

        for (uint8_t j = 0; j < 8; j++) {
            // Algorytm LSB (odwrócony) - standard Modbus/ARC
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001; // Wielomian
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
