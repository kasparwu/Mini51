/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 4 $
 * $Date: 13/10/07 3:51p $ 
 * @brief    This file gives a demo to access EEPROM by software I2C master.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/ 
#include <stdio.h>
#include "Mini51Series.h"
#include "i2c_software_gpio_with_timer.h"

void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
   /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to XTL, STCLK to XTL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_XTAL | CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */        
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk;
    
    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_XTAL;

/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP = SYS_MFP_P01_RXD | SYS_MFP_P00_TXD;
    
    /* Lock protected registers */
    SYS_LockReg();    
    
    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void UART_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART_RST_Msk;
    
     /* Configure UART and set UART Baudrate */
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (((__XTAL + (115200/2)) / 115200)-2);    
    UART->LCR = 0x3 | (0x0 << UART_LCR_PBE_Pos) | (0x0 << UART_LCR_NSB_Pos) ;
}

int32_t main (void)
{
    uint8_t Tx_Data[6];
    
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();
    
    printf("+-------------------------------------------------------+\n");
    printf("|    Software I2C sample code                           |\n");
    printf("|    SW Master -> I2C EEPROM                            |\n");
    printf("+-------------------------------------------------------+\n");

    I2C_SW_I_Open(50000);
    Tx_Data[0]=0;
    Tx_Data[1]=0;
    Tx_Data[2]=0xAA;
    Tx_Data[3]=0xBB;
    Tx_Data[4]=0x55;
    Tx_Data[5]=0xCC;

    printf("Write data into EEPROM\n");
    printf("Data:0x%x,0x%x,0x%x,0x%x\n",Tx_Data[2],Tx_Data[3],Tx_Data[4],Tx_Data[5] );
    I2C_SW_I_Send(0x50,Tx_Data,6);
    while(I2C_SW_I_IsBZ());
    if(I2C_SW_I_Count()!=6)
        while(1);
    CLK_SysTickDelay(5000);

    printf("Write address into EEPROM\n");
    I2C_SW_I_Send(0x50,Tx_Data,2);
    while(I2C_SW_I_IsBZ());
    if(I2C_SW_I_Count()!=2)
        while(1);
        
    printf("Read data form EEPROM\n");        
    I2C_SW_I_Get(0x50,Tx_Data,4);
    while(I2C_SW_I_IsBZ());
    printf("Data:0x%x,0x%x,0x%x,0x%x\n",Tx_Data[0],Tx_Data[1],Tx_Data[2],Tx_Data[3] );
    if(I2C_SW_I_Count()!=4)
        while(1);
    while(1);
}
