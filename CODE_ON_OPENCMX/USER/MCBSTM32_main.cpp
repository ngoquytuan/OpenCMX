/*******************************************************
This program was created by tuannq

Project : Test on MCBSTM32 kit
Version : 1
Date    : 15/06/2018
Author  : tuannq
Company : None
Comments: 


Chip type               : STM32F103RBT6
Program type            : Examples
External Clock frequency: 8.000000 MHz
*******************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "GPIO_STM32F10x.h"
#include "main.h"

#define LED(state) GPIO_PinWrite(GPIOA, 11, state);

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar] = {0x5555, 0x6666, 0x7777};


// Check mcu clock
RCC_ClocksTypeDef mcu_clk;



__IO uint32_t TimingDelay = 0;
__IO uint32_t task1s = 1000;
//UART1 receiver timeout
__IO uint32_t u1out = 50;// 50 is 50ms
/******************RTC***********************************************/

__IO uint32_t TimeDisplay = 0;
uint8_t USART_Scanf(uint32_t value);

/******************RTC***********************************************/


///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//ErrorStatus  HSEStartUpStatus;
//FLASH_Status FlashStatus;
////uint16_t VarValue = 0;
///* Virtual address defined by the user: 0xFFFF value is prohibited */
//uint16_t VirtAddVarTab[NumbOfVar] = {0x5555, 0x6666, 0x7777};
//uint16_t eeprom_temp;


// USART Receiver buffer
const uint8_t RX_BUFFER_SIZE0=20;
uint16_t USART1_Time_Out; 
uint8_t USART1_index=0,USART1_rx_data_buff[RX_BUFFER_SIZE0];
uint8_t USART1_Time_Out_Over, USART1_Time_Out_Running,USART1_process;


void TIM2_IRQHandler(void)
{
		if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

			//for USART1 rec time out
			if(USART1_Time_Out_Running==1)
			{
				USART1_Time_Out--;
				if(USART1_Time_Out==0) USART1_Time_Out_Over=1;
			}
		}
}

void Delay(__IO uint32_t nTime);


int main(void)
{
  
	SystemInit();
	SystemCoreClockUpdate();
	/* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
	
	//GPIO config
	GPIO_PortClock   (GPIOC , true);
	//TX1 RD485
	GPIO_PortClock   (GPIOB, true);
	GPIO_PinConfigure(GPIOB, 8, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT10MHZ);
	GPIO_PinWrite(GPIOB, 8, 1);
	//GPIO_PortWrite(GPIOB, 8,0);
	//RL1
	GPIO_PortClock   (GPIOB, true);
	GPIO_PinConfigure(GPIOB, 9, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT10MHZ);
	GPIO_PinWrite(GPIOB, 9, 0);
	//RL8
	//GPIO_PortClock   (GPIOA, true);
	GPIO_PinConfigure(GPIOA, 5, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT10MHZ);
	GPIO_PinWrite(GPIOA, 5, 0);
	
  /* Check if the system has resumed from WWDG reset */
  if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)
  { 
    /* WWDGRST flag set */
    /* Turn on LED1 */
    GPIO_PinWrite(GPIOB, 9, 1);
    /* Clear reset flags */
    RCC_ClearFlag();
  }
  else
  {
    /* WWDGRST flag is not set */
    /* Turn off LED1 */
    GPIO_PinWrite(GPIOB, 9, 0);
  }
	
	
  

	//sw_eeprom_stm32();
	
	USART1_Init();
	delay_ms(1);
	printf("Kiem tra uart 1\r\n");
	
	
	//test_eeprom();
	//check clock CPU
	RCC_GetClocksFreq(&mcu_clk);
	printf(">Thach anh: \r\nADCCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\nSYSCLK:%d",mcu_clk.ADCCLK_Frequency,mcu_clk.HCLK_Frequency,mcu_clk.PCLK1_Frequency,mcu_clk.PCLK2_Frequency,mcu_clk.SYSCLK_Frequency);
	
	//LCD_Init();
	//LCD_Clear();
	//LCD_Puts("Sometime it last");
	//LCD_Gotoxy(1,0);
	//LCD_Puts(" in love!");
	
	/***********************RTC*********************************/
  //RTC_Init();
	
	/* WWDG configuration */
  //WWDG_Init();
	
	while(1)
	{
		/* Update WWDG counter */
			WWDG_SetCounter(127);
		/* If 1s has been elapsed */
//    if (TimeDisplay == 1)
//    {
//      /* Display current time */
//      Time_Display(RTC_GetCounter());
//      TimeDisplay = 0;
//    }
//		if(task1s == ONTIME)
//		{
//			task1s = 1000;
//			printf("Kiem tra 1s\r\n");
//		}
		if(u1out == ONTIME)
		{
			u1out = STOP;// Da nhan du ban tin UART => Xy ly
			printf("UART1:%s\r\n",USART1_rx_data_buff);
			for(USART1_index=0;USART1_index<RX_BUFFER_SIZE0;USART1_index++)
                            {
                            USART1_rx_data_buff[USART1_index]=0;
                            }  
                            USART1_index=0;
		}
	}


	
		
}


	
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  while(TimingDelay != 0)
  {
  }
}


	
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

///**
//  * @brief  Retargets the C library printf function to the USART.
//  * @param  None
//  * @retval None
//  */
// PUTCHAR_PROTOTYPE
//{
//	/* Place your implementation of fputc here */
//	/* e.g. write a character to the USART */
//	/* Loop until the end of transmission */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//	USART_SendData(USART1, (uint8_t) ch);

//	

//	return ch;
//}

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
  {}
}

#endif	



