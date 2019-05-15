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
//接收状怿
//bit15＿接收完成标志
//bit14＿接收刿x0d
//bit13~0＿接收到的有效字节数目
u16 USART_MC20E_RX_STA;       //接收状态标访

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


//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//u8 USART_MC20E_RX_BUF[USART_MC20E_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字芿

/**
函数名：void Usart_MC20E_init(u32 bound)
简介：MC20E串口初始化
参数：bound：波特率
返回值：
*/  
void Usart_MC20E_init(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);	//GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_USART_MC20E, ENABLE);//使能USART_MC20E
    
    //USART_MC20E_TX   GPIOB.10
    GPIO_InitStructure.GPIO_Pin = USART_MC20E_TX_PIN; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(USART_MC20E_GPIO, &GPIO_InitStructure);//初始化GPIOA.2
    
    //USART_MC20E_RX	  GPIOB.11
    GPIO_InitStructure.GPIO_Pin = USART_MC20E_RX_PIN;//PB.11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(USART_MC20E_GPIO, &GPIO_InitStructure);//初始化GPIOA.3  
    
    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART_MC20E_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先线
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存噿
    
    //USART 初始化设罿
    
    USART_InitStructure.USART_BaudRate = bound;//串口波特玿
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长丿位数据格弿
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    
    USART_Init(USART_MC20E, &USART_InitStructure); //初始化串叿
    USART_ITConfig(USART_MC20E, USART_IT_RXNE, ENABLE);//开启串口接受中斿
   
    
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  	//使能串口2的DMA发送
    UART_DMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)MC20E_Tx_Buff);//DMA1通道2,外设为串口3,存储器为USART3_TX_BUF 
     
    USART_Cmd(USART_MC20E, ENABLE);                    //使能串口1
    
 //   TIM4_Init(99,7199);		//10ms中断
    USART_MC20E_RX_STA=0;		//接收数据计数器清零
    //TIM4_Set(0);			//关闭定时器4
}

void USART_MC20E_CLR_Buf(void)                           // 串口缓存清理
{
    memset(MC20E_Rx_Buff, 0, USART_MC20E_MAX_RX_LEN);      //清空
    USART_MC20E_RX_STA =0;
    //USART3_RX_STA =0;
    MC20E_Rx_Count = 0;                    
}

///////////////////////////////////////USART3 DMA发送配置部分//////////////////////////////////	   		    
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址    
 void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
    DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
    DMA_InitStructure.DMA_BufferSize = 0;  //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器	
} 
/**
函数名：static void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
简介：开启一次DMA传输
参数：
返回值：
*/
 void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
{
    DMA_Cmd(DMA_CHx, DISABLE );  //关闭 指示的通道        
    DMA_SetCurrDataCounter(DMA_CHx,len);//DMA通道的DMA缓存的大小	
    DMA_Cmd(DMA_CHx, ENABLE);           //开启DMA传输
}	   

/**
函数名：void USART3_IRQHandler(void) 
简介：串口3中断服务函数
参数：
返回值：
*/
void USART3_IRQHandler(void)                	
{
    u8 Res;
    
    if(USART_GetITStatus(USART_MC20E, USART_IT_RXNE) != RESET)//接收中断(接收到的数据必须昿x0d 0x0a结尾)
    {
	Res =USART_ReceiveData(USART_MC20E);//读取接收到的数据
	
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
        
//	if(GSM_typedef.GSM_Receive_Correct)//数据头正确 ，开始接受服务器数据
//	{
//            GSM_typedef.MC20E_Ready =0;  
//            
//	    if( MC20E_Rx_Buff [MC20E_Rx_Count] == 'a') 
//            {
//                if( MC20E_Rx_Buff [MC20E_Rx_Count -2] == 'd') //数据以回车换行结尾
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
//  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
//  {	 
//    res =USART_ReceiveData(USART3);	
////    if(res == '+')    //进入命令模式
////    {
////        symble_order_mode =1;
////    }
//    // 		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//还可以接收数据
//    // 		{
//    // 			TIM_SetCounter(TIM4,0);//计数器清空        				 
//    // 			if(USART2_RX_STA==0)TIM4_Set(1);	 	//使能定时器4的中断 
//    // 			USART2_RX_BUF[USART2_RX_STA++]=res;		//记录接收到的值	 
//    // 		}else 
//    // 		{
//    // 			USART2_RX_STA|=1<<15;					//强制标记接收完成
//    // 		} 
//    if(MC20E_Rx_Count<USART_MC20E_MAX_RX_LEN)		//还可以接收数据
//    {
//      TIM_SetCounter(TIM4,0);//计数器清空        				 
//      if(USART_MC20E_RX_STA==0)TIM4_Set(1);	 	//使能定时器4的中断 
//      MC20E_Rx_Buff[MC20E_Rx_Count++]=res;		//记录接收到的值	 
//    } 
//  }


//_Bool Deal_ordor(char *str)
//{
//    if(strstr(MC20E_Rx_Buff,str)!=NULL) 
//        return 1;
//    else return 0;
//}

//void USART3_IRQHandler(void)                	//串口3中断服务程序
//{
//    u8 Res;
//    
//    if(USART_GetITStatus(USART_MC20E, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须昿x0d 0x0a结尾)
//    {
//	Res =USART_ReceiveData(USART_MC20E);	//读取接收到的数据	
//	
//	if((USART_MC20E_RX_STA&0x8000)==0)//接收未完房
//	{
//	    if(USART_MC20E_RX_STA&0x4000)//接收到了0x0d
//	    {
//		if(Res!=0x0a)USART_MC20E_RX_STA=0;//接收错误,重新开姿
//		else {
//		    
//		    USART_MC20E_RX_STA|=0x8000;	//接收完成亿
//		    //Rx_Count_2G = USART_MC20E_RX_STA;
//		    USART_MC20E_RX_STA = 0;		    	    
//		}
//		
//	    }
//	    else //还没收到0X0D
//	    {	
//		if(Res==0x0d)USART_MC20E_RX_STA|=0x4000;
//		else
//		{
//		    MC20E_Rx_Buff[USART_MC20E_RX_STA&0X3FFF]= Res ;		    
//		    USART_MC20E_RX_STA++;		   
//		    if(USART_MC20E_RX_STA>(USART_MC20E_MAX_RX_LEN-1))USART_MC20E_RX_STA=0;//接收数据错误,重新开始接收	
//		} 			
//	    }	   
//	}			 
//    } 
//}

//串口3,printf 函数
//确保一次发送数据不超过USART2_MAX_SEND_LEN字节
void u3_printf(char* fmt,...)  
{  
  va_list ap;
  va_start(ap,fmt);
  vsprintf((char*)MC20E_Tx_Buff,fmt,ap);
  va_end(ap);
  while(DMA_GetCurrDataCounter(DMA1_Channel2)!=0);	//等待通道2传输完成   
  UART_DMA_Enable(DMA1_Channel2,strlen((const char*)MC20E_Tx_Buff)); 	//通过dma发送出去
}

////定时器4中断服务程序		    
//void TIM4_IRQHandler(void)
//{ 	
//  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//更新中断
//  {	 			   
//    USART_MC20E_RX_STA|=1<<15;	//标记接收完成
//    TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志    
//    TIM4_Set(0);			//关闭TIM4  
//  }	    
//}
//设置TIM4的开关
//sta:0，关闭;1,开启;
void TIM4_Set(u8 sta)
{
  if(sta)
  {
    TIM_SetCounter(TIM4,0);//计数器清空
    TIM_Cmd(TIM4, ENABLE);  //使能TIMx	
  }else TIM_Cmd(TIM4, DISABLE);//关闭定时器4	   
}
//通用定时器中断初始化
//这里始终选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数		 
void TIM4_Init(u16 arr,u16 psc)
{	
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能//TIM4时钟使能    
    
    //定时器TIM3初始化
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

              
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
}