
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 13/10/07 3:57p $
 * @brief    Timer sample for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

void TMR0_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete\n");
    TIMER_ClearIntFlag(TIMER0); 
    
}

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
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

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

    /* Set P3 multi function pin for Timer 0 counter pin */
    SYS->P3_MFP = SYS_MFP_P34_T0;
    
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int i;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART, 115200);

    printf("\nThis sample code use timer 0 to count P0.0 input event\n");
    printf("Please connet P0.0 to P3.4, press any key to continue\n");
    getchar();

    P0->DOUT |= 1;                     // Set init state to high
    P0->IMD = (P0->IMD & ~0x3) | 0x1;  // Set to output mode
    
    // Give a dummy target frequency here. Will over write prescale and compare value with macro 
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);
    
    // Update prescale and compare value to what we need in event counter mode.
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 1000);
    // Counter increase on falling edge
    TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_FALLING_EDGE);
    // Start Timer 0
    TIMER_Start(TIMER0);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    
    
    for(i = 0; i < 1000; i++) {
        P0->DOUT &= ~1; // low
        P0->DOUT |= 1;  // high
    }
        
    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


