
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 13/10/07 3:59p $ 
 * @brief    Template project for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/   
#include <stdio.h>
#include "Mini51Series.h"
#include "incfile.h"


void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz XTAL (UART), and internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk | CLK_APBCLK_TMR1_EN_Msk; //Add Timer1 by Kenny.

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;
    
    /* Set P3 multi-function pins for Timer toggle output pin */
    SYS->P3_MFP = SYS_MFP_P34_T0 | SYS_MFP_P35_T1;	//Add by Kenny

    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;
    
    /* Lock protected registers */
    SYS_LockReg();         
}    

void Timer_Init(void)
{
    /* To generate 500HZ toogle output, timer frequency must set to 1000Hz.
       Because toggle output state change on every timer timeout event */
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, Timer_Constant);
    TIMER_Start(TIMER0);

	TIMER_Open(TIMER1,TIMER_TOGGLE_MODE,Timer_Constant*2);
	TIMER_Start(TIMER1);
}

void Timer_update(void)
{
	TIMER_Stop(TIMER0);
	TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, Timer_Constant);
    TIMER_Start(TIMER0);

	TIMER_Stop(TIMER1);
	TIMER_Open(TIMER1,TIMER_TOGGLE_MODE,Timer_Constant*2);
	TIMER_Start(TIMER1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_IRQHandler(void)
{
    UART_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_HANDLE()
{
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART->ISR;

    if(u32IntSts & UART_ISR_RDA_INT_Msk)
    	{
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART)) 
        	{

			if(UART_ReadCounter>2) 
				{
				for(UART_ReadCounter=1;UART_ReadCounter<3;UART_ReadCounter++)
					{
					if((UART_Buffer[UART_ReadCounter]>='0')&&(UART_Buffer[UART_ReadCounter]<='9'))
						UART_Buffer[UART_ReadCounter]=UART_Buffer[UART_ReadCounter]-0x30;
					else if((UART_Buffer[UART_ReadCounter]>='A')&&(UART_Buffer[UART_ReadCounter]<='F'))
						UART_Buffer[UART_ReadCounter]=UART_Buffer[UART_ReadCounter]-0x37;
					else if((UART_Buffer[UART_ReadCounter]>='a')&&(UART_Buffer[UART_ReadCounter]<='f'))
						UART_Buffer[UART_ReadCounter]=UART_Buffer[UART_ReadCounter]-0x57;
					else
						UART_Buffer[UART_ReadCounter]=0;
					}			
				UART_Buffer[3]=(UART_Buffer[1]<<4)+(UART_Buffer[2]);
				Timer_Constant=UART_Buffer[3];
				Timer_update();

				SPI_WRITE_TX(SPI,UART_Buffer[3]);
				SPI_TRIGGER(SPI);
            	while(SPI_IS_BUSY(SPI));

				printf(" \n Input Valuse is 0x%x \n",UART_Buffer[3]);

				UART_Buffer[4]=SPI_READ_RX(SPI);
;
				printf(" \n Receive Valuse is 0x%x \n",UART_Buffer[4]);
				
				UART_ReadCounter=0;
				Hello_Message();
				}
			else
				{
				UART_Buffer[UART_ReadCounter] = UART_READ(UART);		   // 讀取輸入字元 
				UART_WRITE(UART,UART_Buffer[UART_ReadCounter]);		  // 接收到的字元送回uart
				UART_ReadCounter++;
				}
            /* Check if buffer full */
	        if(g_u32comRbytes < RXBUFSIZE)
	        	{
	             /* Enqueue the character */
	             g_u8RecData[g_u32comRtail] = u8InChar;
	             g_u32comRtail = (g_u32comRtail == (RXBUFSIZE-1)) ? 0 : (g_u32comRtail+1);
	             g_u32comRbytes++;
	        	}           
	        	
    		if(u32IntSts & UART_ISR_THRE_INT_Msk)
	    		{
	    		uint16_t tmp;
	        	tmp = g_u32comRtail;
	        	if(g_u32comRhead != tmp)
	        		{
	        		u8InChar = g_u8RecData[g_u32comRhead];
	            	//UART_WRITE(UART,u8InChar);
	            	g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
	            	g_u32comRbytes--;
	        		}
    			}
        	}
    	}
}

void UART_Enable()
{
    UART_ENABLE_INT(UART, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RTO_IEN_Msk));
    NVIC_EnableIRQ(UART_IRQn);
	
}

void UART_Disable()
{
    UART_DISABLE_INT(UART, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RTO_IEN_Msk));
    NVIC_DisableIRQ(UART_IRQn);
	
}

void Hello_Message()
{
	printf("=========================\n");
	printf("=LePower_SPI=\n");
	printf("=========================\n\n");
	
	printf("Vsync_N (2xNx30Hz, N=0~4) : 1'h");
}

void SPI_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init SPI                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI, SPI_MASTER, SPI_MODE_0, 32, 2000000);
	SPI_ENABLE_BYTE_REORDER(SPI);	

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);
}


int main()
{
    
    SYS_Init();
	SPI_Init();
	Timer_Init();

    UART_Open(UART, 115200);
	Hello_Message();   

	UART_Enable();

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
