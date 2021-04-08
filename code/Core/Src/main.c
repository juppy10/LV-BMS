/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "registers.h"
#include "functions.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
    HAL_StatusTypeDef I2CStat;
  /* USER CODE END 1 */

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
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void writeRegister(uint8_t address, uint8_t data){
    uint8_t buf[3];        //is it meant to be uint or char??
    HAL_StatusTypeDef I2CStat;
    buf[0] = address; buf[1] = data;

    I2CStat = HAL_I2C_Master_Transmit(&hi2c1,BQ_I2CADDRESS,buf,2,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    //Add CRC later
}
int readRegister(uint8_t address){
    uint8_t buf = address;
    HAL_StatusTypeDef I2CStat;

    HAL_I2C_Master_Transmit(&hi2c1,BQ_I2CADDRESS,&buf,1,HAL_MAX_DELAY);
    I2CStat = HAL_I2C_Master_Receive(&hi2c1,BQ_I2CADDRESS,buf,1,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    return buf[0];
}
long setShortCircuitProtection(long current_mA){
    //need to check notebook
    uint8_t PROTECT_1;
    PROTECT_1 = (uint8_t) readRegister(PROTECT1);
    PROTECT_1 |= (SHORTCIRCUIT_THRESHOLD << SCD_THRESH);
    PROTECT_1 |= (SHORTCIRCUIT_DELAY << SCD_DELAY);
    writeRegister(PROTECT1, PROTECT_1);
}
long setOvercurrentDischargeProtection(long current_mA){
    //need to check notebook
    uint8_t PROTECT_2;
    PROTECT_2 = (uint8_t) readRegister(PROTECT2);
    PROTECT_2 |= (OVERCURRENT_THRESHOLD << OCD_THRESH);
    PROTECT_2 |= (OVERCURRENT_DELAY << OCD_DELAY);
    writeRegister(PROTECT2, PROTECT_2);
}
int setCellUndervoltageProtection(int voltage_mV, int delay_s) {
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S, uv_trip;
    int8_t adc_offset;
    uint8_t protecc3Reg, delay_s_code, protecc3New;

    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = (uint8_t) ((ADCGAIN_1 << 1)|(ADCGAIN_2 >> 5)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits

    adc_offset = (int8_t) readRegister(ADCOFFSET);

    uv_trip = (((voltage_mV - adc_offset) * 1000/ADCGAIN_S) >> 4) & 0x00FF; //need to check
    uv_trip += 1;

    writeRegister(UV_TRIP, uv_trip);

    protecc3Reg = readRegister(PROTECT3) & 0x3F;    //preserve first 6 LSB, set UV bits to 0

    if(delay_s > 16){
        delay_s = 16;
    }//if the delay is greater than 16, change it to 16
    delay_s_code = delay_s / 4;     //quotient with 4
    delay_s_code = (delay_s_code << 6);     //left shift by 6 (to make the 2LSB bits 6 and 7)
    protecc3New = delay_s_code | protecc3Reg;   //or with existing register

    writeRegister(PROTECT3,protecc3New);    //write to register
}
int setCellOvervoltageProtection(int voltage_mV, int delay_s){
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S, ov_trip;
    int8_t adc_offset;
    uint8_t protecc3Reg, delay_s_code, protecc3New;

    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = ((ADCGAIN_1 << 1)|(ADCGAIN_2 >> 5)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits

    adc_offset = (int8_t) readRegister(ADCOFFSET);

    ov_trip = (((voltage_mV - adc_offset) * 1000/ADCGAIN_S) >> 4) & 0x00FF; //need to check

    writeRegister(OV_TRIP, ov_trip);

    protecc3Reg = readRegister(PROTECT3) & 0xCF;    //preserve bits 4 and 5, set OV bits to 0

    if(delay_s > 16){
        delay_s = 16;
    }//if the delay is greater than 16, change it to 16
    delay_s_code = delay_s / 4;     //quotient with 4
    delay_s_code = (delay_s_code << 4);     //left shift by 6 (to make the 2LSB bits 7 and 8)
    protecc3New = delay_s_code | protecc3Reg;   //or with existing register

    writeRegister(PROTECT3,protecc3New);    //write to register
}
uint16_t getBatteryVoltage(void){
    uint8_t HI_BYTE, LO_BYTE;
    uint16_t BAT_Volt;

    HI_BYTE = readRegister(BAT_HI_BYTE);
    LO_BYTE = readRegister(BAT_LO_BYTE);

    BAT_Volt = HI_BYTE << 8 | LO_BYTE;
    return BAT_Volt;
}
//long  getBatteryCurrent(void){}   //might not need it

uint8_t getGain() {
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S;
    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = ((ADCGAIN_1 << 1u)|(ADCGAIN_2 >> 5u)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits
    return ADCGAIN_S;

}

// TAKES ARRAY OF LENGTH "NUMBER_OF_CELLS" AND UPDATES THE VALUES IN THE ARRAY
int updateCellVoltages(int *voltages) {
    for (uint8_t i = 0; i < NUMBER_OF_CELLS; i++) {
        voltages[i] = getCellVoltage(i + 1); // CELL ID STARTS AT 1
    }
    return 0;
}

int  getCellVoltage(int idCell) {
    // V(cell) = GAIN x ADC(cell) + OFFSET
    uint8_t v_hi, v_lo;
    v_hi = (uint8_t) readRegister(CELL_ADDRESS(idCell));
    v_lo = (uint8_t) readRegister(CELL_ADDRESS(idCell) + 1);
    long adc = (v_hi & 0b00111111u) << 8 | v_lo;
    int8_t adc_offset = readRegister(ADCOFFSET);
    uint8_t gain = getGain();
    return adc * gain / 1000 + adc_offset;
}
int  getMinCellVoltage(void){

}

int  getMaxCellVoltage(void){

}

float getTemperatureDegC(void){

}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
