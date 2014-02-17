/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 13/10/07 3:51p $ 
 * @brief    This file gives a demo using 24LC64 library to access EEPROM flash by I2C.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/  
#include <stdio.h>
#include <string.h>
#include "Mini51Series.h"
#include "LCD_Driver.h"
#include "EEPROM_24LC64.h"

uint8_t gu8Count = 0, isPress = FALSE;
uint8_t g_u8Buf[256] = {0};

void delay_loop(void)
{
    uint32_t i, j;
    for (i = 0;i < 3;i++)
    {
        for (j = 0;j < 60000;j++);
    }
}

void EINT0_IRQHandler(void)
{
    P3->ISRC = 1 << 2;
    gu8Count++;
    isPress = TRUE;
}

void GPIO_Init(void)
{
    /* Enable debunce function of P3.2 (EINT0) */
    GPIO_ENABLE_DEBOUNCE(P3, 2);

    /* Set debounce time. it is about 6.4 ms */
    GPIO->DBNCECON = GPIO_DBNCECON_DBCLKSRC_IRC10K | GPIO_DBNCECON_DBCLKSEL_64;

    /* Enable P3.2 to be EINT0 */
    GPIO_EnableInt(P3, 2, GPIO_INT_RISING);
    NVIC_EnableIRQ(EINT0_IRQn);
}

int32_t I2C_24LC64_AutoTest(void)
{
    int32_t i, i32Err;

    /* Programming EEPROM */
    for(i=0;i<256;i++)
        EEPROM_Write(i, i);

    /* Verify */
    i32Err = 0;
    for(i=0;i<256;i++)
    {
        if(EEPROM_Read(i) != i)
        {
            i32Err = 1;
            break;
        }
    }

    LCD_ClearScreen();
    if(i32Err)
    {    
        LCD_Print(0, "I2C EEPROM");
        LCD_Print(1, "Write Fail");
        return -1;
    }
    else
    {
        LCD_Print(0, "I2C EEPROM");
        LCD_Print(1, "Verify OK!");
    }
    
    /* Delay for 2 seconds */
    for(i=0;i<20;i++)
        CLK_SysTickDelay(100000);

    EEPROM_SequentialRead(0, g_u8Buf, 256);
    /* Verify */
    i32Err = 0;
    for(i=0;i<256;i++)
    {
        if(g_u8Buf[i] != i)
        {
            i32Err = 1;
            break;
        }
    }

    LCD_ClearScreen();
    if(i32Err)
    {    
        LCD_Print(0, "I2C EEPROM");
        LCD_Print(1, "Seq. Read Fail");
        return -1;
    }
    else
    {
        LCD_Print(0, "I2C EEPROM");
        LCD_Print(1, "Seq. Read OK!");
    }

    /* Delay for 2 seconds */
    for(i=0;i<20;i++)
        CLK_SysTickDelay(100000);

    for(i=0;i<256;i++)
        g_u8Buf[i] = i;
    for(i=0;i<8;i++)
        EEPROM_PageWrite(i * 32, &g_u8Buf[i*32]);    

    memset(g_u8Buf, 0, 256);

    EEPROM_SequentialRead(0, g_u8Buf, 256);
    /* Verify */
    i32Err = 0;
    for(i=0;i<256;i++)
    {
        if(EEPROM_Read(i) != (i & 0xFF))
        {
            i32Err = 1;
            break;
        }
    }

    LCD_ClearScreen();
    if(i32Err)
    {    
        LCD_Print(0, "I2C EEPROM");
        LCD_Print(1, "Page Write Fail");
        return -1;
    }
    else
    {
        LCD_Print(0, "I2C EEPROM");
        LCD_Print(1, "Page Write OK!");
    }

    return i32Err;

}

int32_t I2C_24LC64_ManualTest(void)
{
    uint32_t i2cdata = 0, temp;
    char addr[16] = "Address:";
    char Write[16] = "Write:";
    char read[16] = "Read:";

    LCD_Print(0, "I2C with 24LC65");
    LCD_Print(1, "test read and  ");
    LCD_Print(2, "write function ");
    LCD_Print(3, "press INT button");


    temp = 0x55;
    while (1)
    {
        if (isPress)
        {
            isPress = FALSE;
            switch (gu8Count)
            {
            case 1:
                LCD_ClearScreen();
                LCD_Print(0, "Key1 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 11);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 11);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 2:
                LCD_ClearScreen();
                LCD_Print(0, "Key2 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 22);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 22);
                sprintf((void *)&read[5], "%x", i2cdata);            
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 3:
                LCD_ClearScreen();
                LCD_Print(0, "Key3 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 33);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 33);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 4:
                LCD_ClearScreen();
                LCD_Print(0, "Key4 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 44);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 44);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 5:
                LCD_ClearScreen();
                LCD_Print(0, "Key5 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 55);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 55);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 6:
                LCD_ClearScreen();
                LCD_Print(0, "Key6 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 66);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 66);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 7:
                LCD_ClearScreen();
                LCD_Print(0, "Key7 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 77);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 77);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 8:
                LCD_ClearScreen();
                LCD_Print(0, "Key8 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 88);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 88);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                break;
            case 9:
                LCD_ClearScreen();
                LCD_Print(0, "Key9 had pressed ");
                EEPROM_Write(0x00000000 + temp, temp + 99);
                i2cdata = EEPROM_Read(0x00000000 + temp);
                sprintf((void *)&addr[8], "%x", temp);
                sprintf((void *)&Write[6], "%x", temp + 99);
                sprintf((void *)&read[5], "%x", i2cdata);
                LCD_Print(1, addr);
                LCD_Print(2, Write);
                LCD_Print(3, read);
                gu8Count = 0;
                break;
            default:
                break;
            } //End Switch
        } //End if(isPress)
    } //End While

}

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
    
    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    /* Lock protected registers */
    SYS_LockReg();
    
    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

int main(void)
{    
     uint32_t i;
        
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();     

    /* Init GPIO P3.2 as EINT0 */
    GPIO_Init();
    
    LCD_Init();  
    LCD_EnableBackLight();
    LCD_ClearScreen();

   /* 
       This sample code should work with EEPROM 24LC64
       to show how to program EEPROM through I2C interface.

       The demo will program EEPROM and verfy the written data.
       Finally, user may press "SW_INT" key to write and read a byte.
       And the byte will shown on LCD display 
    */

    EEPROM_Init();

    /* Test EEPROM read/write automatically */
    I2C_24LC64_AutoTest();

    /* Delay for 2 seconds */
    for(i=0;i<20;i++)
        CLK_SysTickDelay(100000);
    
    /* Test EEPROM read/write by key pressing */
    I2C_24LC64_ManualTest();
}

