/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 13/10/07 3:52p $ 
 * @brief    MINI51 series software I2C implement source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/   
#include <stdio.h>
#include "Mini51Series.h"

#include "i2c_software_gpio.h"
#include "i2c_hardware.h"

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
    uint8_t Rx_Data[6];
    
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();
    
    printf("+-------------------------------------------------------+\n");
    printf("|    Software I2C sample code                           |\n");
    printf("|    SW Master                                          |\n");
    printf("|    HW Slave                                           |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Initiate I2C for slave\n");
    InitI2C_HW();
    
    printf("Initiate software I2C for Master\n");
    I2C_SW_Open(100000);

    printf("Please connect:\n");
    printf("SW SDA - P1.4 (PIN47)  <->   HW SDA - P3.4 (PIN9) \n");
    printf("SW SCL - P1.5 (PIN2)   <->   HW SCL - P3.5 (PIN10) \n");
    
    Tx_Data[0]=0;
    Tx_Data[1]=0;
    Tx_Data[2]=0xA5;
    Tx_Data[3]=0xcc;
    Tx_Data[4]=0xbb;
    Tx_Data[5]=0xdd;

    printf("Access I2C slave:\n");
    
    I2C_SW_Send(0x15,Tx_Data,6);
    CLK_SysTickDelay(5000);
    I2C_SW_Get(0x15,Rx_Data,4);

    if((Rx_Data[0] != 0xaa) || (Rx_Data[1] != 0x22) || (Rx_Data[2] != 0x33) || (Rx_Data[3] != 0x44)) 
        printf("Data Error!!\n");
    else
        printf("Pass!!\n");    
        
    return 0;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
