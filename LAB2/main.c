/******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

char lcd_buffer[6];    // LCD display buffer



bool button_push;
uint16_t Tim2_PrescalerValue, Tim3_PrescalerValue,Tim4_PrescalerValue;
TIM_HandleTypeDef    Tim2_Handle, Tim3_Handle,Tim4_Handle;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type
uint16_t count = 2; //count
char* time[5];
uint16_t BestResult; //best time
uint32_t randtime = 1; //random time
int statevar = 0; //state
uint16_t user_time = 0; //time to push button
uint16_t EECHECK; //check if eeprom is empty
RNG_HandleTypeDef Rng_Handle;

__IO uint16_t Tim3_CCR;




/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
	HAL_Init();
	//EE_WriteVariable(VirtAddVarTab[0], NULL);
  /* Configure the system clock to 4 MHz */
  SystemClock_Config();
	TIM2_Config();
	TIM3_Config();
	TIM4_Config();
	Tim3_CCR = 1000;
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Off(LED4);

	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);

	//BSP_LCD_GLASS_ScrollSentence((uint8_t*) "  mt3ta4 lab2 starter", 1, 200);
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	








	
//******************* use emulated EEPROM ====================================
	//First, Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }


	

// then can write to or read from the emulated EEPROM


	
	
	
	
	
	
	
	
	
//*********************use RNG ================================  
Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
//then can use RNG

//HAL_RNG_GenerateRandomNumber(&Rng_Handle, &randtime);

	

  /* Infinite loop */
  while (1)
  {
		

	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  // The following clock configuration sets the Clock configuration sets after System reset                
  // It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig 
  // and to be eventually adapted to new clock configuration                                               

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz

void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    Tim3 is of 16 bits. Timer 2..7 is on APB1.
	
		Since the clock source is MSI, and the clock range is RCC_MSIRANGE_6, SystemCoreClock=4Mhz.
	
		Since RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1, HCLK=4Mhz.
	
		Since RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1, PCLK1=4MHz, and TIM3CLK=4Mhz.
		(if the APB1 prescaler is not 1, the timer clock frequency will be 2 times of APB1 frequency)
	
		 that is, for current RCC config, the the Timer3 frequency=SystemCoreClock.
	
		To get TIM3's counter clock at 10 KHz, for example, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	
		i.e: Prescaler = (SystemCoreClock /10 KHz) - 1
       
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 1000000) - 1; //input 1 for to get 1 hz //overflow to reset oc ccr
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
 
  Tim3_Handle.Init.Period = 1000 - 1;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}

void TIM2_Config(void){
	Tim2_PrescalerValue = (uint16_t) (SystemCoreClock/ 1) - 1;
	
	Tim2_Handle.Instance = TIM2;
	
	Tim2_Handle.Init.Period = 2000;
  Tim2_Handle.Init.Prescaler = Tim2_PrescalerValue;
  Tim2_Handle.Init.ClockDivision = 0;
  Tim2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim2_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim2_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}

// configure Timer4 base.
void  TIM4_Config(void)
{
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock / (1/randtime)) - 1; //RANDTIME frequency

  /* Set TIM3 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = randtime; // Arr register (ovrflow) for timer 4, want timer 3
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}



/*void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR; //
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 /*	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}

*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		 //SEL_JOY_PIN    			
								statevar ++;
								if(statevar == 1){
								button_push = true;
								}
								//button_down = false;
								//count = 0;
			/* Toggle LED4 */
							//BSP_LED_Toggle(LED4);
							//BSP_LED_Toggle(LED4);
						//	BSP_LED_Toggle(LED5);
						//	BSP_LED_Toggle(LED5);
							//BSP_LCD_GLASS_DisplayChar((uint8_t *)'A', singlePoint, doublePoint, 2);
							//BSP_LCD_GLASS_Clear();
							//BSP_LCD_GLASS_DisplayString((uint8_t*)"select");		

							//factor = (factor+1)%2;
							//__HAL_TIM_SET_COMPARE(&Tim4_Handle, TIM_CHANNEL_1, Tim4_CCR/(factor+1));
							break;	
			case GPIO_PIN_1: //LEFT_JOY_PIN  
							/* Toggle LED4 */
							//BSP_LED_Toggle(LED4);
							//BSP_LCD_GLASS_DisplayChar((uint8_t *)'B', singlePoint, doublePoint, 3);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"left");
							break;
			case GPIO_PIN_2: //RIGHT_JOY_PIN  
							/* Toggle LED4 */
							//BSP_LED_Toggle(LED4);
							//BSP_LCD_GLASS_DisplayChar((uint8_t *)'B', singlePoint, doublePoint, 3);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"right");			
							break;
			case GPIO_PIN_3:  //UP_JOY_PIN 
							/* Toggle LED4 */
							//BSP_LED_Toggle(LED4);
							//BSP_LCD_GLASS_DisplayChar((uint8_t *)'B', singlePoint, doublePoint, 3);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"up");
							break;
			
			case GPIO_PIN_5:  //DOWN_JOY_PIN
							//button_down = true;
							//button_push = false;
							//count = 0;
							/* Toggle LED4 */
							//BSP_LED_Toggle(LED5);
							//BSP_LCD_GLASS_DisplayChar((uint8_t *)'B', singlePoint, doublePoint, 3);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"down");
							break;
			default://
						//default
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)"OTHER");
	  } 
}



//write logic code here
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //see  stm32fxx_hal_tim.c for different callback function names. 
																													//for timer 3 , Timer 3 use update event initerrupt
{
	
	if ((*htim).Instance==TIM2 && statevar == 0){    
		BSP_LED_Toggle(LED5); //BLINK LED AT START
	}

	
	if(statevar == 1 && button_push == true){ //AFTER USER PUSH BUTTON, GENERATE RANDOM NUMBER AND START TIMER 4
		BSP_LED_Off(LED5);
		HAL_RNG_GenerateRandomNumber(&Rng_Handle, &randtime);
			
		for (int i = 12; i < 33; i++){
			randtime &= ~(1<<i);
		} //makes randtime reasonable time
    //BSP_LCD_GLASS_Clear();
		//sprintf(lcd_buffer, "%d", randtime);
		//BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
		button_push = false;
		BSP_LCD_GLASS_Clear();
	}
		
	
	if((*htim).Instance == TIM3 && statevar == 1){ //TIMING HOW LONG IT TAKES USER TO PUSH BUTTON
	user_time++;
	}

  if(user_time == randtime){
    BSP_LED_On(LED5); //TURN ON LED AFTER RANDTIME
		/*sprintf(lcd_buffer, "%d", user_time);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);*/
  }
	
	if(statevar == 2){
		sprintf(lcd_buffer, "%d", statevar);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
		
    if(user_time < randtime){ //IF PUSHED BEFORE THEN RESTART GAME
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString("CHEAT");
      //BSP_LCD_GLASS_ScrollSentence((uint8_t*) "YOU ARE A CHEATER!", 1, 200);
      statevar = 0;
    }

    EE_ReadVariable(VirtAddVarTab[1], &EECHECK); //CHECK TO SEE IF EEprom has been written to
    if(EECHECK != true){
      BestResult = user_time - randtime;
      EE_WriteVariable(VirtAddVarTab[0], BestResult);
      EE_WriteVariable(VirtAddVarTab[1], true);
      statevar = 3;
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"works");
  }
    //Printing "Best Time" text to the screen
    EE_ReadVariable(VirtAddVarTab[0], &BestResult); //READ BEST TIME
	if(EECHECK == true){
		if(user_time < BestResult){ //CHECK IF TIME IS LESS
			BestResult = user_time - randtime; //STORE USER TIME IN BEST RESULT
			EE_WriteVariable(VirtAddVarTab[0], BestResult); //WRITE BEST RESULT TO EEPROM
      statevar = 3; //INCREMENT TO NEXT STATE
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"works");
    }
  }
    
    //Printing best time value to the screen //FLASHES TOO FAST, NEED TO FLASH SLOWER
    if(statevar == 3){ //FLASHES BEST RESULT
      sprintf(lcd_buffer, "%d", BestResult); //
      if(count%2 == 0){
        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_ScrollSentence((uint8_t*) "Best Time: ", 1, 200);
      }
      else{
        BSP_LCD_GLASS_Clear();
		    BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
      }
		  count++;
    }

   
      
    }
  }

	


//logic end

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
		if ((*htim).Instance==TIM4){
	//if (button_push == false & button_down == false){
			 BSP_LED_Toggle(LED5); //Change this to 5 for part 1
	}
		//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h

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
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif