/****** 



 1. both OD mode and PP mode can drive the motor! However, some pin can not output  high in OD mode!!! 
   (maybe because those pins have other alternate functions)). 

 2. the signals do not need to be inverted before feeded in H-bridge! 
*/




#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);

__IO uint16_t Tim3_CCR;
TIM_HandleTypeDef    Tim3_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure;
uint16_t Tim3_PrescalerValue;

static GPIO_InitTypeDef  GPIO_InitStruct;

static void SystemClock_Config(void);
static void Error_Handler(void);
void fullcw(int phase);
void fullccw(int phase);
void halfcw(int phase);
void halfccw(int phase);
void GPIO_Config(void);
static void EXTILine3_Config(void);
void ExtBtn2_Config(void);
static void EXTILine1_Config(void);
void  TIM3_Config(void);
void  TIM3_OC_Config(void);
enum DirectionState { clockwise,counterclockwise
};
enum Stepping{half, full
};

enum DirectionState direction = clockwise;
enum Stepping SteppingMode=full;
int phase=0;

int main(void){
	
		/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
		HAL_Init();
		
	
		 /* Configure the system clock to 180 MHz */
		SystemClock_Config();
		Tim3_CCR=11875;
		HAL_InitTick(0x0000); // set systick's priority to the highest.
//		void GPIO_Config();
    EXTILine3_Config();
    ExtBtn2_Config();
    EXTILine1_Config();
    TIM3_Config();
    TIM3_OC_Config();

		BSP_LCD_Init();
		//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
		BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
															// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
		//BSP_LCD_SelectLayer(uint32_t LayerIndex);
		BSP_LCD_SelectLayer(0);
		//BSP_LCD_SetLayerVisible(0, ENABLE); //do not need this line.
		BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
		BSP_LCD_DisplayOn();
	 
		BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
	
		LCD_DisplayString(2, 3, (uint8_t *)"Lab");
	
		LCD_DisplayInt(2, 8, 5);
			
			GPIO_InitTypeDef GPIO_InitStruct;
//	 __HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		
		
		while(1) {	

			
		} // end of while loop
	
}  //end of main


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}





void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
		if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
		{
				switch(SteppingMode){
						case full:
							SteppingMode=half;
							phase=0;
							Tim3_CCR=5936;
							break;
						case half:
							SteppingMode=full;
							phase=0;
							Tim3_CCR=11875;
							break;
						default:
							break;
					}
		}
		
		
		if(GPIO_Pin == GPIO_PIN_1)
		{
			  Tim3_CCR -= 3000;
		}  //end of PIN_1

		if(GPIO_Pin == GPIO_PIN_2)
		{
				 Tim3_CCR += 3000;
				
		} //end of if PIN_2	
		
		if(GPIO_Pin == GPIO_PIN_3)
		{
					
					switch(direction){
						case clockwise:
							direction=counterclockwise;
							break;
						case counterclockwise:
							direction=clockwise;
							break;
						default:
							break;
					}
				
				
		} //end of if PIN_23
}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
			
				if ((*htim).Instance==TIM3) {
					switch(SteppingMode){
						case full:
							switch(direction){
						    case clockwise:
									
		            LCD_DisplayString(3, 4, (uint8_t *)"full_cw");
								  if(phase!=3){
										phase++;
									}
									else{
										phase=0;
									}
									fullcw(phase);
									break;
								case counterclockwise:
								  LCD_DisplayString(3, 4, (uint8_t *)"full_ccw");
									if(phase!=3){
										phase++;
									}
									else{
										phase=0;
									}
									fullccw(phase);
									break;
							break;
							}
					  case half:
							switch(direction){
						    case clockwise:
									LCD_DisplayString(3, 4, (uint8_t *)"half_cw");
									if(phase!=7){
										phase++;
									}
									else{
										phase=0;
									}
									halfcw(phase);
									break;
								case counterclockwise:
									LCD_DisplayString(3, 4, (uint8_t *)"hall_ccw");
									if(phase!=7){
										phase++;
									}
									else{
										phase=0;
									}
									halfccw(phase);
									break;
							}
						  break;
					}
				}
				
			
	
}
 
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */

void fullcw(int phase)
{
	switch (phase) 
	{
		case 0: 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			break;
		
		case 1:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); 
			  break;
		
		case 2: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
			  break;
		
		case 3: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			  break;
	}
}
void fullccw(int phase)
{
	switch ( phase) 
	{
		case 0: 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			break;
		
		case 1:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); 
			  break;
		
		case 2: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
			  break;
		
		case 3: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			  break;
	}
}


void halfcw(int phase)
{
	switch (phase)
	{
case 0: 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			break;
		
		case 1:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); 
			  break;
		
		case 2: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			  break;
		
		case 3: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
			  break;
		
		case 4:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);  
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);  
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);  
			break;
		
		case 5: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); 
			  break;
		
		case 6: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);  
			  break;
		
		case 7: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);  
			  break;
	}
}

void halfccw(int phase)
{
	switch (phase)
	{
		case 0: 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			break;
		
		case 1:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); 
			  break;
		
		case 2: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			  break;
		
		case 3: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
			  break;
		
		case 4:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);  
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); 
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);  
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);  
			break;
		
		case 5: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);   
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET); 
			  break;
		
		case 6: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);  
			  break;
		
		case 7: 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);  
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); 
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);  
			  break;
	}
}

static void EXTILine1_Config(void) //PC1
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PC1 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}


void ExtBtn2_Config(void){  // pin PD2 on the board
 
	GPIO_InitTypeDef   GPIO_InitStructure;
  /* Enable GPIOB clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

static void EXTILine3_Config(void) 
{
  GPIO_InitTypeDef   GPIO_InitStructure; //pc3

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PC1 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void GPIO_Config(void){
	
//	GPIO_InitTypeDef GPIO_InitStruct;
//	 __HAL_RCC_GPIOE_CLK_ENABLE();
//	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

//	GPIO_InitStruct.Pin = GPIO_PIN_2;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//	
//	GPIO_InitStruct.Pin = GPIO_PIN_3;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//	
//	
//	GPIO_InitStruct.Pin = GPIO_PIN_4;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//	
//	GPIO_InitStruct.Pin = GPIO_PIN_5;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
}

void  TIM3_Config(void)
{
  Tim3_PrescalerValue = 17999;
  Tim3_Handle.Instance = TIM3; 
	
  Tim3_Handle.Init.Period =11875;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
}
void  TIM3_OC_Config(void)
{
		Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim3_OCInitStructure.Pulse=Tim3_CCR;
		Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		HAL_TIM_OC_Init(&Tim3_Handle);
		HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1);
	 	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1);
				
		
}
