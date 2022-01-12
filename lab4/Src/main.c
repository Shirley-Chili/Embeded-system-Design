/****** 


1.The Fist Extern button (named extBtn1)  connected to PC1, ExtBtn1_PC1_Config  //
		2014: canot use pin PB1, for 429i-DISCO ,pb1 is used by LCD. if use this pin, always interrupt by itself
					can not use pin PA1, since it is used by gyro. if use this pin, never interrupt.
					pd1----WILL ACT AS PC13, To trigger the RTC timestamp event
					....ONLY PC1 CAN BE USED TO FIRE EXTI1 !!!!
2. the Second external button (extBtn2)
		2014: 
		PA2: NOT OK. (USED BY LCD??)
		PB2: ????.
		PC2: ok, BUT sometimes (every 5 times around), press pc2 will trigger exti1, which is configured to use PC1. (is it because of using internal pull up pin config?)
		      however, press PC1 does not affect exti 2. sometimes press PC2 will also affect time stamp (PC13)
		PD2: OK,     
		PE2:  OK  (PE3, PE4 PE5 , seems has no other AF function, according to the table in manual for discovery board)
		PF2: NOT OK. (although PF2 is used by SDRAM, it affects LCD. press it, LCD will flick and displayed chars change to garbage)
		PG2: OK
		

*/

#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)

/* Timer handler declaration */
TIM_HandleTypeDef    Tim3_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure;

// When the prescaler was set at 180, 1 ms was every 500 ticks of TIM3
// Therefore, for a prescaler of 1800, 1 ms will be evry 50 ticks of TIM3 
uint16_t TIM3Prescaler=1799;

// For a prescaler of 1800, a 1000 ticks is 20 ms
uint16_t TIM3Period=1000;    

// Setting the PW for the 4 respective channels
// The time period is 1000, therfore the CCR values must be less than a 1000 for an interupt to occur
__IO uint16_t TIM3_CCR1_Val=200, TIM3_CCR2_Val=400, TIM3_CCR3_Val=600, TIM3_CCR4_Val=800;  
 
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

__IO uint16_t ADC3ConvertedValue=0;


 volatile double  setPoint=23.5;  //NOTE: if declare as float, when +0.5, the compile will give warning:
															//"single_precision perand implicitly converted to double-precision"
															//ALTHOUGH IT IS A WARNING, THIS WILL MAKE THE PROGRAM not WORK!!!!!!
															//if declare as float, when setPoint+=0.5, need to cast : as setPioint+=(float)0.5, otherwise,
															//the whole program will not work! even this line has  not been used/excuted yet
															//BUT, if declare as double, there is no such problem.
															
	    														
	//			Project Options -> Target -> Floating Point Hardware -> Use FPU

	//You must then have code to enable the FPU hardware prior to using any FPU instructions. This is typically in the ResetHandler or SystemInit()

	//            system_stm32f4xx.c
  ////            void SystemInit(void)
	//							{
	//								/* FPU settings ------------------------------------------------------------*/
	//								#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	//									SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//								#endif
	//							...											
	//		-----MODIFY the system_stm32f4xx.c in the above way, will also fix the "float" type problem mentioned above. 												
	//   the is noted in 2016. by 2021, this problem may be fixed by STM.											

double measuredTemp;


// Configuring TIM3 
void TIM3_PWM_Config(void);

void  LEDs_Config(void);
void  LEDs_On(void);
void  LEDs_Off(void);
void  LEDs_Toggle(void);

// Configuring 2 external buttons required
void ExtBtn1_Config(void);  // Pin PC1 on the board
void ExtBtn2_Config(void);	// Pin PD2 on the board

void ADC_DMA_Config(void);

// Temparature sensor and required variables
float ConvertADC3Value(void);
void InitialSetPointTemp(void);
// InitialSet = 1 when a setPoint temp has been set (later on in the code)
int InitialSet = 0; 

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);

static void SystemClock_Config(void);
static void Error_Handler(void);

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
		
		HAL_InitTick(0x0000); // set systick's priority to the highest.
	
		//Configure LED3 and LED4 ======================================
		LEDs_Config();
	  // Turning them off since they are not needed
	  LEDs_Off();
	
		//configure the USER button as exti mode. 
		BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
																			
	  ExtBtn1_Config();  // Pin PC1 on the board 
		ExtBtn2_Config();	 // Pin PD2 on the board
	
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
	
	
		LCD_DisplayString(3, 2, (uint8_t *) "Lab4 Starter ");
	
		LCD_DisplayString(9, 0, (uint8_t *) "Current ");
		LCD_DisplayString(10, 0, (uint8_t *)"setPoint");
	  
		// Displaying the current setPoint temperature threshold value
		LCD_DisplayFloat(9, 10, setPoint, 2);
		LCD_DisplayFloat(10, 10, setPoint, 2);
		
		ADC_DMA_Config();
		TIM3_PWM_Config();
		
	// Using the examples provided as reference	
	while(1) {		
		
		// Initial setPoint temp is only set once there is a non-zero reading from the sensor
		if (!InitialSet && ConvertADC3Value() > 0) { 
			InitialSetPointTemp();
		}
		
		LCD_DisplayFloat(9, 10, ConvertADC3Value(), 2);
		LCD_DisplayFloat(10, 10, setPoint, 2);
		
		// if the recorde temp exceeds the setPoint threshold the the FAN is turned on
		if (ConvertADC3Value() > setPoint) { 
						
			// Fan intitially starts at 50% speed plus 
			// Plus the difference between the current and setPoint temps multiplied by 50
      // 50x gave us a reasonable speed change 
			TIM3_CCR2_Val = 500 + (ConvertADC3Value() - setPoint) * 50; // 50x
			
			/* Displaying fan power percentage on LCD */
			
			LCD_DisplayString	(12, 0, (uint8_t *) "Fan Power: ");
			
			float FanPower = TIM3_CCR2_Val / 10;
			
			// Setting a power cieling 100%
			if (FanPower > 100) {
				FanPower = 100;
			}
		
		  LCD_DisplayFloat	(12, 11, FanPower, 0);
			
			if (FanPower < 100) {
				LCD_DisplayString	(12, 13, (uint8_t *) " ");
			}
			// Percentage symbol
			LCD_DisplayString	(12, 14, (uint8_t *) "%");
			
			/**********************************************/
			
			// Checking the temp \
			__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_2,TIM3_CCR2_Val);
		
		}
		// Turning fan off
		else { 
			TIM3_CCR2_Val = 0; 
			// Displaying a power of 0%
			LCD_DisplayString	(12, 0, (uint8_t *) "Fan Power: 00 %");
			__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_2,TIM3_CCR2_Val);  
		}	
		
	} // end of while loop
	
}  //end of main

	// Timer 3 PWM Configuration
void  TIM3_PWM_Config(void)
{
	/*******to make this PWM works with the sevo motor in the future,
 the cycle period will be designed as 20 ms
TIM3 TIM4 are 16 bits, TIM2 and TIM5 are 32 bits	
	TIM2---5 are on APB1 bus, which is 45MHZ
since the APB1 prescaller is not 1, the timer Freq is 2XFreq_of_APB1, that is 90Mhz.
so to make a cyle as 0.02 s, the Period	should be: 0.02/(1/90M)=1800,000, which is larger than allwed MAX period value 65536
so need to use a prescaler for the timer: if set the prescaller as 1800 (assign 1799 to the register), then the period can be 1000
*/
	
	
		/* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
   
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
   
  Tim3_Handle.Init.Period = TIM3Period;;  
  Tim3_Handle.Init.Prescaler = TIM3Prescaler;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim3_Handle.Init.RepetitionCounter = 0;  //default is 0
	
	Tim3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
 
 
	if(HAL_TIM_PWM_Init(&Tim3_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
	// PWM Config
	Tim3_OCInitStructure.OCMode=  TIM_OCMODE_PWM1; //TIM_OCMODE_TIMING;
	Tim3_OCInitStructure.OCFastMode=TIM_OCFAST_DISABLE;
	Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
	//Tim3_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH //complementary polarity. 
																	//This parameter is valid only for TIM1 and TIM8.
	//Tim3_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET; //This parameter is valid only for TIM1 and TIM8.
  //Tim3_OCInitStructure.OCNIdleState= TIM_OCNIDLESTATE_RESET; //This parameter is valid only for TIM1 and TIM8.
	
	Tim3_OCInitStructure.Pulse=TIM3_CCR1_Val;   // Value of 200
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1) != HAL_OK) // Channel 1
  {
    /* Configuration Error */
    Error_Handler();
  }
	
	
	Tim3_OCInitStructure.Pulse=TIM3_CCR2_Val;  // Value of 400
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2) != HAL_OK) // Channel 2
  {
    /* Configuration Error */
    Error_Handler();
	}
	
	
	Tim3_OCInitStructure.Pulse=TIM3_CCR3_Val;   // Value of 600;
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_3) != HAL_OK) // Channel 3
  {
    /* Configuration Error */
    Error_Handler();
	}
	
	
	Tim3_OCInitStructure.Pulse=TIM3_CCR4_Val;   // Value of 800;
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_4) != HAL_OK) // Channel 4
  {
    /* Configuration Error */
    Error_Handler();
	}
	
	
	//------------------------------------------------------------------------------------------------------

	
if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	
if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	
if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	
if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	
}



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


// Conversion function for temp sensor 
// Discussed in lab, PDF attached for reference
float ConvertADC3Value(void) {
	return ADC3ConvertedValue / 40.95;
}
// Initializing the setPoint threshold value
void InitialSetPointTemp(void) {
	// Adding 1 so The starting temperature is be higher than the starting room temp
	setPoint = ConvertADC3Value() + 1.0; 
	InitialSet = 1;
}
// ADC Confg
void ADC_DMA_Config(void) {
	
	ADC_ChannelConfTypeDef sConfig;
	
	AdcHandle.Instance          = ADC3;
  
  AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode = DISABLE;
  AdcHandle.Init.ContinuousConvMode = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 0;
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion = 1;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;
  AdcHandle.Init.EOCSelection = DISABLE;
      
  if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
	/*##-2- Configure ADC regular channel ######################################*/  
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfig.Offset = 0;
  
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler(); 
  }
	
	/*##-3- Start the conversion process and enable interrupt ##################*/
  /* Note: Considering IT occurring after each number of ADC conversions      */
  /*       (IT by DMA end of transfer), select sampling time and ADC clock    */
  /*       with sufficient duration to not create an overhead situation in    */
  /*        IRQHandler. */ 
  if(HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&ADC3ConvertedValue, 1) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler(); 
	}
	
}

void LEDs_Config(void)
{
 /* Initialize Leds mounted on STM32F429-Discovery board */
	BSP_LED_Init(LED3);   //BSP_LED_....() are in stm32f4291_discovery.c
  BSP_LED_Init(LED4);
}

void LEDs_On(void){
/* Turn on LED3, LED4 */
 BSP_LED_On(LED3);
  BSP_LED_On(LED4);
}

void LEDs_Off(void){
/* Turn on LED3, LED4 */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
}
void LEDs_Toggle(void){
/* Turn on LED3, LED4 */
  BSP_LED_Toggle(LED3);
  BSP_LED_Toggle(LED4);
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



/* Button Configuration Functions */
void ExtBtn1_Config(void)     // for GPIO C pin 1
// can only use PA0, PB0... to PA4, PB4 .... because only  only  EXTI0, ...EXTI4,on which the 
	//mentioned pins are mapped to, are connected INDIVIDUALLY to NVIC. the others are grouped! 
		//see stm32f4xx.h, there is EXTI0_IRQn...EXTI4_IRQn, EXTI15_10_IRQn defined
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
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





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
  {
		LEDs_Toggle();	
			
  }
	
	
	if(GPIO_Pin == GPIO_PIN_1)
  {
		
		LEDs_Toggle();
		// Incrementing the setPoint with each button press
		setPoint += 0.5; 

	}  //end of PIN_1

	if(GPIO_Pin == GPIO_PIN_2)
  {
		LEDs_Toggle();
		// Decrementing the setPoint with each button press
		setPoint -= 0.5;
			
	} //end of if PIN_2	
	
	
}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	//	if ((*htim).Instance==TIM4)
			 //BSP_LED_Toggle(LED4);
		//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
	
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM3_pwm
	
	//__HAL_TIM_SET_COUNTER(htim, 0x0000);   //not necessary  
}



static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

	
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and 
  *         you can add your own implementation.    
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* Turn LED3 on: Transfer process is correct */
  BSP_LED_On(LED3);
	
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
