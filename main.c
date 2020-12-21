/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t TxBuff[SIZE];
uint8_t RxBuff[SIZE];
uint8_t Read_Reg[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t status_register = 0;
uint8_t status_register_after = 0;
uint8_t ItStatus1 = 0;
uint8_t ItStatus2 = 0;
uint8_t device_status = 0;
uint8_t recived = 0;
uint8_t payload[11];
uint8_t rx = 0;
uint8_t i = 0;



uint8_t addr[1] = {0x00};
uint8_t receive_nigger_buff[4];


int buffer_length = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SpiWriteRegister(uint8_t reg_adress, uint8_t reg_value);
uint8_t SpiReadRegister(uint8_t adress);
void SystemClock_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SpiWriteRegister(uint8_t reg_adress, uint8_t reg_value){
	uint8_t SPI_Write_Buffer[2];

	SPI_Write_Buffer[0] = reg_adress | 0x80;
	SPI_Write_Buffer[1] = reg_value;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0); //select card
	HAL_SPI_Transmit(&hspi1, SPI_Write_Buffer, SIZE, 15);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1); //deselect card
}

uint8_t SpiReadRegister(uint8_t adress)
{
   uint8_t SPI_Adress_Dummy[2] = {0, 0xff};
	 uint8_t buffer[2] = {0,0};
   SPI_Adress_Dummy[0] = adress & 0x7F;
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0); //select card
	 HAL_SPI_TransmitReceive(&hspi1, SPI_Adress_Dummy, buffer, 2, 500);
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1); //deselect card
	 return buffer[1];
}

void config()
{
	
	SpiWriteRegister(0x07, 0x80); //write 0x80 to the Operating & Function Control1 register
	//wait for chip ready interrupt from the radio (while the nIRQ pin is high)
 // while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5));
   HAL_Delay(100);
	
	//read interrupt status registers to clear the interrupt flags and release NIRQ pin
	ItStatus1 = SpiReadRegister(0x03); //read the Interrupt Status1 register
	ItStatus2 = SpiReadRegister(0x04); //read the Interrupt Status2 register
	
	//set the center frequency to 433 MHz
	SpiWriteRegister(0x75, 0x53); //write 0x75 to the Frequency Band Select register  +
	SpiWriteRegister(0x76, 0x4B); //write 0xBB to the Nominal Carrier Frequency1 register +
	SpiWriteRegister(0x77, 0x00); //write 0x80 to the Nominal Carrier Frequency0 register +
	
	/*set the modem parameters according to the excel calculator (parameters: 12 kbps, deviation: 6t (45) kHz, channel filter
        BW: 47.7t (112.1) kHz*/
	SpiWriteRegister(0x1C, 0x1E); //write 0x2Dt to the IF Filter Bandwidth register (2*45k+12k)
	SpiWriteRegister(0x20, 0xD0); //write 0x53t to the Clock Recovery Oversampling Ratio register	???
	SpiWriteRegister(0x21, 0x00); //write 0x01t to the Clock Recovery Offset 2 register
	SpiWriteRegister(0x22, 0x9D); //write 0x89t to the Clock Recovery Offset 1 register
	SpiWriteRegister(0x23, 0x49); //write 0x37t to the Clock Recovery Offset 0 register
	SpiWriteRegister(0x24, 0x00); //write 0x03t to the Clock Recovery Timing Loop Gain 1 register
	SpiWriteRegister(0x25, 0x24); //write 0x18t to the Clock Recovery Timing Loop Gain 0 register
	SpiWriteRegister(0x1D, 0x00); //write 0x40h | 0x3Ct to the AFC Loop Gearshift Override register
	SpiWriteRegister(0x72, 0x48); //t write 0x1F | 0x48h to the Frequency Deviation register
 
 
 
  //TANSMITTER CONFIG
	
	
  SpiWriteRegister(0x6E, 0x4E); //write 0x4E to the TXDataRate 1 register
  SpiWriteRegister(0x6F, 0xA5); //write 0xA5 to the TXDataRate 0 register
  SpiWriteRegister(0x70, 0x2C); //write 0x2C to the Modulation Mode Control 1 register
  //set the desired TX deviation (+-45 kHz)
  SpiWriteRegister(0x72, 0x48); //write 0x48 to the Frequency Deviation register
  SpiWriteRegister(0x34, 0x1E); //write 0x09 to the Preamble Length register ----------------------------------------------------PREAMBLE LENGTH
 	SpiWriteRegister(0x6D, 0x18 | 0x00); //min power, vanos detectori vegar agiqvams, unda shemowmdes status registeri 
 	SpiWriteRegister(0x3E, 0x01); //write packet length 1 byte; lets make it parametrizeble 

	//  SpiWriteRegister(0x30, 0x0D); //write 0x0D to the Data Access Control register

	
	//TANSMITTER CONFIG

	/*Configure the receive packet handler*/
	//Disable header bytes; set variable packet length (the length of the payload is defined by the
	//received packet length field of the packet); set the synch word to two bytes long
	SpiWriteRegister(0x33, 0x02); //write 0x02 to the Header Control2 register
	
	//Disable the receive header filters
	SpiWriteRegister(0x32, 0x00); //write 0x00 to the Header Control1 register
	//Set the sync word pattern to 0x2DD4
	SpiWriteRegister(0x36, 0x2D); //write 0x2D to the Sync Word 3 register
	SpiWriteRegister(0x37, 0xD4); //write 0xD4 to the Sync Word 2 register
	//Enable the receive packet handler and CRC-16 (IBM) check
	SpiWriteRegister(0x30, 0x85); //write 0x85t to the Data Access Control register, for Tx its 0D for this register
	//Enable FIFO mode and GFSK modulation
	SpiWriteRegister(0x71, 0x23); //write 0x63h to the Modulation Mode Control 2 register
	//set preamble detection threshold to 20bits
	SpiWriteRegister(0x35, 0x15); //write 0x30 to the Preamble Detection Control register

	SpiWriteRegister(0x7E, 0x00); // RX FIFO 0 byte 
	
	//antenna switch control
	SpiWriteRegister(0x0B, 82); //write 0x52 GPIO0 tx	0x12h
	SpiWriteRegister(0x0C, 85); //write 0x55 GPIO1 rx	0x15h


	/*enable receiver chain*/
	SpiWriteRegister(0x07, 0x05);//write 0x05 to the Operating Function Control 1 register
	// THIS WILL BE USED TO SWITCH TO TRANSMIT
	//Enable two interrupts:
	// a) one which shows that a valid packet received: 'ipkval'
	// b) second shows if the packet received with incorrect CRC: 'icrcerror'
	SpiWriteRegister(0x05, 0x07); //write 0x03 to the Interrupt Enable 1 register
	SpiWriteRegister(0x06, 0xE0); //write 0x00 to the Interrupt Enable 2 register
	//read interrupt status registers to release all pending interrupts
	ItStatus1 = SpiReadRegister(0x03);//read the Interrupt Status1 register
	ItStatus2 = SpiReadRegister(0x04);//read the Interrupt Status2 register	

	SpiWriteRegister(0x27, 0xAA); // RSSI threshold
  SpiWriteRegister(0x08, 0x00);//write 0x00 to the Operating Function Control 2 register
	
}



void transmit(uint8_t txData) {
  
 //set the length of the payload to 8 bytes
  SpiWriteRegister(0x3E, 0x01); //write 8 to the Transmit Packet Length register
  //SpiWriteRegister(0x30, 0x0D); //write 0x0D to the Data Access Control register
  //fill the payload into the transmit FIFO
  SpiWriteRegister(0x7F, txData);
  //SpiWriteRegister(0x7F, txData);
 	SpiWriteRegister(0x30, 0x0D); //write 0x85t to the Data Access Control register, for Tx its 0D for this register


  //Disable all other interrupts and enable the packet sent interrupt only.
  //This will be used for indicating the successful packet transmission for the MCU
  SpiWriteRegister(0x05, 0x04); //write 0x04 to the Interrupt Enable 1 register
  SpiWriteRegister(0x06, 0x00); //write 0x03 to the Interrupt Enable 2 register

  //Read interrupt status registers. It clear all pending interrupts and the nIRQ pin goes back to high.
  ItStatus1 = SpiReadRegister(0x03); //read the Interrupt Status1 register
  ItStatus2 = SpiReadRegister(0x04); //read the Interrupt Status2 register

  /*enable transmitter*/
  //The radio forms the packet and send it automatically.
  SpiWriteRegister(0x07, 0x09); //write Tx on to the Operating Function Control 1 register

  /*wait for the packet sent interrupt*/
  //The MCU just needs to wait for the 'ipksent' interrupt.
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5));

  //read interrupt status registers to release the interrupt flags
	/*
	if(SpiReadRegister(0x03) & (1 << 2))
  {
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		 HAL_Delay(2000);
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		 HAL_Delay(2000);
	}
	it worked
	*/
  ItStatus1 = SpiReadRegister(0x03); //read the Interrupt Status1 register
  ItStatus2 = SpiReadRegister(0x04); //read the Interrupt Status2 register
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

}

void receive(){
	

	

	
	/*enable receiver chain*/
	SpiWriteRegister(0x07, 0x05);//write 0x05 to the Operating Function Control 1 register
	// THIS WILL BE USED TO SWITCH TO TRANSMIT
	//Enable two interrupts:
	// a) one which shows that a valid packet received: 'ipkval'
	// b) second shows if the packet received with incorrect CRC: 'icrcerror'
	SpiWriteRegister(0x05, 0x07); //write 0x03 to the Interrupt Enable 1 register
	SpiWriteRegister(0x06, 0xE0); //write 0x00 to the Interrupt Enable 2 register
	//read interrupt status registers to release all pending interrupts
	ItStatus1 = SpiReadRegister(0x03);//read the Interrupt Status1 register
	ItStatus2 = SpiReadRegister(0x04);//read the Interrupt Status2 register	
	
	
		SpiWriteRegister(0x30, 0x85); //write 0x85t to the Data Access Control register, for Tx its 0D for this register

	
	  device_status = SpiReadRegister(0x4B); //buffer length
	  status_register = SpiReadRegister(0x26);




    //wait for the interrupt event
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5));

    //read interrupt status registers
    ItStatus1 = SpiReadRegister(0x03);//read the Interrupt Status1 register
    ItStatus2 = SpiReadRegister(0x04);//read the Interrupt Status2 register


    /*CRC Error interrupt occured*/
    if ( (ItStatus1 & 0x01) == 0x01 )
    {

      //disable the receiver chain
      SpiWriteRegister(0x07, 0x01);//write 0x01 to the Operating Function Control 1 register
      //reset the RX FIFO
      SpiWriteRegister(0x08, 0x02);//write 0x02 to the Operating Function Control 2 register
      SpiWriteRegister(0x08, 0x00);//write 0x00 to the Operating Function Control 2 register
      //print error
      //Serial.println("error");
      //enable the receiver chain again
      SpiWriteRegister(0x07, 0x05);//write 0x05 to the Operating Function Control 1 register
    }

    /*packet received interrupt occurred*/
    if ( (ItStatus1 & 0x02) == 0x02 )
    {
      i++;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

      //disable the receiver chain
			SpiWriteRegister(0x07, 0x01);	//write 0x01 to the Operating Function Control 1 register
			

			//Read the length of the received payload
			//int buffer_length = SpiReadRegister(0x4B);//read the Received Packet Length register
			//check whether the received payload is not longer than the allocated buffer in the MCU
			//if (buffer_length < 11){
				
				//Get the received payload from the RX FIFO
				//for (int temp8 = 0; temp8 < buffer_length; temp8++)
				//{
					payload[0] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[1] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[2] = SpiReadRegister(0x7F);//read the FIFO Access register		
					payload[3] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[4] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[5] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[6] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[7] = SpiReadRegister(0x7F);//read the FIFO Access register	
					payload[8] = SpiReadRegister(0x7F);//read the FIFO Access register
					payload[9] = SpiReadRegister(0x7F);//read the FIFO Access register
					recived = 1;
					

//					payload[8] = SpiReadRegister(0x7F);//read the FIFO Access register


				//}
			//}
			
      
  }
 			//reset the RX FIF
	
		SpiWriteRegister(0x08, 0x02);//write 0x02 to the Operating Function Control 2 register
	  SpiWriteRegister(0x08, 0x00);//write 0x00 to the Operating Function Control 2 register
	  SpiWriteRegister(0x07, 0x05);//write 0x05 to the Operating Function Control 1 register

      
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); //turn on power to the Si4432
	config();
	HAL_Delay(100);
	
	 transmit((uint8_t)0x8D);
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
	  //transmit( (uint8_t)0x89);
	 // HAL_Delay(1);
	  //transmit( (uint8_t)0x65);
		//transmit( (uint8_t)0x89);
		

		// transmit((uint8_t)0x8D);
  
			receive();
		

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
