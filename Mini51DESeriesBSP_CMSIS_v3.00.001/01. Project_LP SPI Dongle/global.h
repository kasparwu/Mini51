//************************************************************************
// FileName:       	Global.h
// Purpose:        	This file contains all the global variables
//------------------------------------------------------------------------
// Edited by:      	Kenny Wu
// Date:           	02/17/2014
// Version:        	Ver. 1.0
//------------------------------------------------------------------------

#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] ={0};
uint8_t g_u8RecData[RXBUFSIZE]  ={0};
volatile uint32_t	UART_Buffer[4]={0};

volatile uint32_t g_u32comRbytes = 1;        
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;
volatile int32_t UART_ReadCounter = 1;
volatile int32_t Timer_Constant = 1000;

