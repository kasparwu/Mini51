//************************************************************************
// FileName:       	Incfile.h
// Purpose:        	This file contains all the included files
//------------------------------------------------------------------------
// Edited by:      	Kenny Wu
// Date:           	02/17/2014
// Version:        	Ver. 1.0
//------------------------------------------------------------------------

#include "global.h"
#include "uart.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_HANDLE(void);
void UART_Enable(void);
void UART_Disable(void);
void Hello_Message(void);
void Timer_Init(void);
void Timer_update(void);
