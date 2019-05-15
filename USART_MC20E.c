/**
******************************************************************************
* @file    USART_MC20E.c 
* @author  Application Team
* @version V1.0.0
* @date    
* @brief   
*          
*          
******************************************************************************
*/
/* Includes -----------------------------------------------------------------*/
#include "USART_MC20E.h"
#include <sys/sys.h>
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "global.h"

#include "MC20E.h"
/* Global variables ---------------------------------------------------------*/
/* External variables -------------------------------------------------------*/
/* External Function --------------------------------------------------------*/
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/

GSM_TypeDef GSM_typedef;
//u8 symble_order_mode =0;

/*
*********************************************************************************************************
*
* Filename      : USART_MC20E.c
* Version       : V1.0.0
* Programmer(s) : 
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*
*/
	

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*
*/
                                                              
#define USART_MC20E                   USART3

#define USART_MC20E_GPIO              GPIOB

#define USART_MC20E_RX_PIN            GPIO_Pin_11

#define USART_MC20E_TX_PIN            GPIO_Pin_10

#define RCC_USART_MC20E               RCC_APB1Periph_USART3

#define RCC_GPIO                      RCC_APB2Periph_GPIOB

#define USART_MC20E_IRQ               USART3_IRQn


/*
*********************************************************************************************************
*                                           EXTERNAL FUNCTIONS
*
*/

extern  void  SCP1000_IntrHook (void);

/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*
*/


/*
***************************************************************************************************
*                                            LOCAL TABLES
*
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*
*/
char MC20E_Tx_Buff[USART_MC20E_MAX_RX_LEN];
 char MC20E_Rx_Buff[USART_MC20E_MAX_RX_LEN];
bool MC20E_Receive_Correct;
uint8_t MC20E_Rx_Count;
//����״��
//bit15�߽�����ɱ�־
//bit14�߽�����x0d
//bit13~0�߽��յ�����Ч�ֽ���Ŀ
u16 USART_MC20E_RX_STA;       //����״̬���

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*
*/

void TIM4_Set(u8 sta);
void TIM4_Init(u16 arr,u16 psc);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*
*/


/*
**********************************************************************************************************
**********************************************************************************************************
**                                         GLOBAL FUNCTIONS
**********************************************************************************************************
**********************************************************************************************************
*/
void Usart_MC20E_init(u32 bound);
void u3_printf(char* fmt,...);
void USART_MC20E_CLR_Buf(void) ;  


//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
//u8 USART_MC20E_RX_BUF[USART_MC20E_REC_LEN];     //���ջ���,���USART_REC_LEN�����e

/**
��������void Usart_MC20E_init(u32 bound)
��飺MC20E���ڳ�ʼ��
������bound��������
����ֵ��
*/  
void Usart_MC20E_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);	//GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_USART_MC20E, ENABLE);//ʹ��USART_MC20E
    
    //USART_MC20E_TX   GPIOB.10
    GPIO_InitStructure.GPIO_Pin = USART_MC20E_TX_PIN; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(USART_MC20E_GPIO, &GPIO_InitStructure);//��ʼ��GPIOA.2
    
    //USART_MC20E_RX	  GPIOB.11
    GPIO_InitStructure.GPIO_Pin = USART_MC20E_RX_PIN;//PB.11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(USART_MC20E_GPIO, &GPIO_InitStructure);//��ʼ��GPIOA.3  
    
    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART_MC20E_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ������
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ懒
    
    //USART ��ʼ�����Z
    
    USART_InitStructure.USART_BaudRate = bound;//���ڲ��ثx
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�دλ���ݸ�
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    
    USART_Init(USART_MC20E, &USART_InitStructure); //��ʼ������
    USART_ITConfig(USART_MC20E, USART_IT_RXNE, ENABLE);//�������ڽ����Д�
   
    
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  	//ʹ�ܴ���2��DMA����
    UART_DMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)MC20E_Tx_Buff);//DMA1ͨ��2,����Ϊ����3,�洢��ΪUSART3_TX_BUF 
     
    USART_Cmd(USART_MC20E, ENABLE);                    //ʹ�ܴ���1
    
 //   TIM4_Init(99,7199);		//10ms�ж�
    USART_MC20E_RX_STA=0;		//�������ݼ���������
    //TIM4_Set(0);			//�رն�ʱ��4
}

void USART_MC20E_CLR_Buf(void)                           // ���ڻ�������
{
    memset(MC20E_Rx_Buff, 0, USART_MC20E_MAX_RX_LEN);      //���
    USART_MC20E_RX_STA =0;
    //USART3_RX_STA =0;
    MC20E_Rx_Count = 0;                    
}

///////////////////////////////////////USART3 DMA�������ò���//////////////////////////////////	   		    
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ    
 void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
    DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
    DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
    DMA_InitStructure.DMA_BufferSize = 0;  //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���	
} 
/**
��������static void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
��飺����һ��DMA����
������
����ֵ��
*/
 void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
{
    DMA_Cmd(DMA_CHx, DISABLE );  //�ر� ָʾ��ͨ��        
    DMA_SetCurrDataCounter(DMA_CHx,len);//DMAͨ����DMA����Ĵ�С	
    DMA_Cmd(DMA_CHx, ENABLE);           //����DMA����
}	   

/**
��������void USART3_IRQHandler(void) 
��飺����3�жϷ�����
������
����ֵ��
*/
void USART3_IRQHandler(void)                	
{
    u8 Res;
    
    if(USART_GetITStatus(USART_MC20E, USART_IT_RXNE) != RESET)//�����ж�(���յ������ݱ���px0d 0x0a��β)
    {
	Res =USART_ReceiveData(USART_MC20E);//��ȡ���յ�������
	
        if(MC20E_Rx_Count < USART_MC20E_MAX_RX_LEN)
        {   
          MC20E_Rx_Buff[MC20E_Rx_Count]= Res ;		    
          MC20E_Rx_Count++;        
        }
        else 
        {
            USART_MC20E_CLR_Buf();
            //MC20E_Rx_Count=0;
        }
        
//	if(GSM_typedef.GSM_Receive_Correct)//����ͷ��ȷ ����ʼ���ܷ���������
//	{
//            GSM_typedef.MC20E_Ready =0;  
//            
//	    if( MC20E_Rx_Buff [MC20E_Rx_Count] == 'a') 
//            {
//                if( MC20E_Rx_Buff [MC20E_Rx_Count -2] == 'd') //�����Իس����н�β
//                {
//                    GSM_typedef.GSM_Rx_Data = MC20E_Rx_Buff;
//                    GSM_typedef.GSM_Rx_Count = MC20E_Rx_Count -1;
//                }
//            } 
//	}
        
	if( GSM_typedef.MC20E_Ready )  
	{
	    if ( MC20E_Rx_Buff[0 ] == 'B') 
            {
                if ( MC20E_Rx_Buff[1 ] == 'B') 
               GSM_typedef.GSM_Receive_Correct = 1;
            }
	    else
            {
            	GSM_typedef.GSM_Receive_Correct = 0;
                MC20E_Rx_Count = 0;
            }
	
	}
    } 
    
}    
    
//  u8 res;	    
//  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
//  {	 
//    res =USART_ReceiveData(USART3);	
////    if(res == '+')    //��������ģʽ
////    {
////        symble_order_mode =1;
////    }
//    // 		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//�����Խ�������
//    // 		{
//    // 			TIM_SetCounter(TIM4,0);//���������        				 
//    // 			if(USART2_RX_STA==0)TIM4_Set(1);	 	//ʹ�ܶ�ʱ��4���ж� 
//    // 			USART2_RX_BUF[USART2_RX_STA++]=res;		//��¼���յ���ֵ	 
//    // 		}else 
//    // 		{
//    // 			USART2_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
//    // 		} 
//    if(MC20E_Rx_Count<USART_MC20E_MAX_RX_LEN)		//�����Խ�������
//    {
//      TIM_SetCounter(TIM4,0);//���������        				 
//      if(USART_MC20E_RX_STA==0)TIM4_Set(1);	 	//ʹ�ܶ�ʱ��4���ж� 
//      MC20E_Rx_Buff[MC20E_Rx_Count++]=res;		//��¼���յ���ֵ	 
//    } 
//  }


//_Bool Deal_ordor(char *str)
//{
//    if(strstr(MC20E_Rx_Buff,str)!=NULL) 
//        return 1;
//    else return 0;
//}

//void USART3_IRQHandler(void)                	//����3�жϷ������
//{
//    u8 Res;
//    
//    if(USART_GetITStatus(USART_MC20E, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ���px0d 0x0a��β)
//    {
//	Res =USART_ReceiveData(USART_MC20E);	//��ȡ���յ�������	
//	
//	if((USART_MC20E_RX_STA&0x8000)==0)//����δ�귿
//	{
//	    if(USART_MC20E_RX_STA&0x4000)//���յ���0x0d
//	    {
//		if(Res!=0x0a)USART_MC20E_RX_STA=0;//���մ���,���¿���
//		else {
//		    
//		    USART_MC20E_RX_STA|=0x8000;	//���������
//		    //Rx_Count_2G = USART_MC20E_RX_STA;
//		    USART_MC20E_RX_STA = 0;		    	    
//		}
//		
//	    }
//	    else //��û�յ�0X0D
//	    {	
//		if(Res==0x0d)USART_MC20E_RX_STA|=0x4000;
//		else
//		{
//		    MC20E_Rx_Buff[USART_MC20E_RX_STA&0X3FFF]= Res ;		    
//		    USART_MC20E_RX_STA++;		   
//		    if(USART_MC20E_RX_STA>(USART_MC20E_MAX_RX_LEN-1))USART_MC20E_RX_STA=0;//�������ݴ���,���¿�ʼ����	
//		} 			
//	    }	   
//	}			 
//    } 
//}

//����3,printf ����
//ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
void u3_printf(char* fmt,...)  
{  
  va_list ap;
  va_start(ap,fmt);
  vsprintf((char*)MC20E_Tx_Buff,fmt,ap);
  va_end(ap);
  while(DMA_GetCurrDataCounter(DMA1_Channel2)!=0);	//�ȴ�ͨ��2�������   
  UART_DMA_Enable(DMA1_Channel2,strlen((const char*)MC20E_Tx_Buff)); 	//ͨ��dma���ͳ�ȥ
}

////��ʱ��4�жϷ������		    
//void TIM4_IRQHandler(void)
//{ 	
//  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//�����ж�
//  {	 			   
//    USART_MC20E_RX_STA|=1<<15;	//��ǽ������
//    TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx�����жϱ�־    
//    TIM4_Set(0);			//�ر�TIM4  
//  }	    
//}
//����TIM4�Ŀ���
//sta:0���ر�;1,����;
void TIM4_Set(u8 sta)
{
  if(sta)
  {
    TIM_SetCounter(TIM4,0);//���������
    TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx	
  }else TIM_Cmd(TIM4, DISABLE);//�رն�ʱ��4	   
}
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��		 
void TIM4_Init(u16 arr,u16 psc)
{	
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��//TIM4ʱ��ʹ��    
    
    //��ʱ��TIM3��ʼ��
    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�

              
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
}