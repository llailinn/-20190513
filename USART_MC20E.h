/*
*********************************************************************************************************
* Filename      : bsp.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*********************************************************************************************************
*/
#ifndef __USART_MC20E_H
#define __USART_MC20E_H


/* INCLUDE FILES---------------------------------------------------------*/
#include "stm32f10x.h"

/* DEFINES---------------------------------------------------------*/


/* INT DEFINES ---------------------------------------------------------*/
#define USART_MC20E_MAX_RX_LEN  			           200  	//定义最大接收字节数 200

#define USART_MC20E_MAX_TX_LEN  			           100  	//定义最大接收字节数 200

/* GLOBAL VARIABLES ---------------------------------------------------------*/


/* MACRO'S ------------------------------------------------------------------*/

/* FUNCTION PROTOTYPES ------------------------------------------------------*/

void Usart_BLE_init(u32 bound);
void USART_MC20E_CLR_Buf(void);  
void u3_printf(char* fmt,...);
//_Bool Deal_ordor(char *str);
void Usart_MC20E_init(u32 bound);
void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar);
void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len);

/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of module include.                               */