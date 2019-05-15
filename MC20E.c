/**
******************************************************************************
* @file    MC20E.c 
* @author  Application Team
* @version V1.0.0
* @date    2019-4-18
* @brief   MC20E集成函数
*          
******************************************************************************/
/* Includes -----------------------------------------------------------------*/
#include "MC20E.h"
#include "stdio.h"
#include "string.h"
#include <delay/delay.h>
#include "USART_MC20E.h"


/* Global variables ---------------------------------------------------------*/
/* External variables -------------------------------------------------------*/
extern char MC20E_Rx_Buff[USART_MC20E_MAX_RX_LEN];
extern char MC20E_Tx_Buff[USART_MC20E_MAX_TX_LEN];
extern u16 MC20E_Rx_Count; 
extern GSM_TypeDef GSM_typedef;    //GSM接收服务器数据存放结构体
extern Multi_Senser_Data  *pMulti_Senser;

/* External Function --------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/


#define ON 1
#define OFF 0
#define GSM_RESET 2
#define GSM_EEROW 3

/* Private typedef ----------------------------------------------------------*/


/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/
 void POWER_MC20_IO(void);
 _Bool POWER_MC20(u8 MODE);

unsigned int send_GPS_Command(char *Command, char *Response, unsigned long Timeout, unsigned char Retry);
static _Bool GET_TCP(void);
static void errorLog(char *eerow,u8 reset_symble);
static void MC20LOW_POWER_INIT(void);
static _Bool GNSS_INIT(void);
static void clrStruct(void);
static _Bool parseGpsBuffer(void);

/* Private variables --------------------------------------------------------*/
_SaveData Save_Data;//GNSS结构体
char TCPServer[] = "103.46.128.49";		//TCP连接的IP地址120.76.205.151
char Port[] = "45669";	     //30333
char CSQ_BUFFER[2]={0};
u8 GPRS_ERROR_NUMBER =0;  //GPRS重连的次数（连接TCP成功，但是数据发送失败）
char GPRS_Send_Buff[20]={0};  //gprs发送BUFFRT
unsigned long  Time_Cont = 0;       //定时器计数器
unsigned int   count = 0;


//MC20低功耗初始化
//参数：无
//返回：
static void MC20LOW_POWER_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	// GPIOB时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  PBout(9) = 0; //先不开启睡眠模式
  
 sendCommand("AT+QSCLK=1\r\n", "OK", 3000, 5) ;//最长等待30s

}

//进入低功耗模式
//参数：无
//返回：无
void MC20_INLOW_POWER(void)
{
  PBout(9) = 1;
  delay_ms(20);
}
//退出低功耗模式
//参数：无
//返回：无
void MC20_OUTLOW_POWER(void)
{
  PBout(9) = 0;
  delay_ms(20);
}


//初始化MC20
//参数：无
//返回：SUCCESS，成功 FAIL失败
//测试通过
_Bool MC20_INIT(void)
{
    u8 times=0;
    u8 res;
    u3_printf("AT\r\n"); 
    delay_ms(100);//等待波特率同步
    while(sendCommand("AT\r\n", "OK", 3000, 10) != SUCCESS)  //最长等待30s
    {
	errorLog("检查模块失败\r\n",GSM_RESET);
	delay_ms(100);
    }
    sendCommand("ATE0\r\n", "OK", 3000, 5);
    while(sendCommand("AT+IPR=115200\r\n", "OK", 3000, 10) != SUCCESS)//固定波特率
    {
	errorLog("AT+IPR波特率固定失败\r\n",GSM_RESET);
	delay_ms(100);
    }
    while(sendCommand("AT+CPIN?\r\n", "READY", 3000, 10) != SUCCESS)  //测试SIM卡
    { 
	errorLog("请检查SIM卡\r\n",GSM_RESET);
	delay_ms(100);
    }
    sendCommand("AT+CREG=1\r\n", "OK", 3000, 2);//发送登陆GSM网码
    sendCommand("AT+CREG=2\r\n", "OK", 3000, 2);
    sendCommand("AT+QVBATT=0,3452\r\n", "OK", 3000, 2);
    
    while(1)   //注册GSM网
    {
	if (sendCommand("AT+CREG?\r\n", "1", 7000, 8) == SUCCESS) 
	{
	    if (sendCommand("AT+CGREG?\r\n", "1", 6000, 8) == SUCCESS) //本地网
		break;//本地网 最长等到50S
	}
	if(sendCommand("AT+CREG?\r\n", "5", 5000, 1) == SUCCESS)  
	{
	    if(sendCommand("AT+CGREG?\r\n", "5", 5000, 1) == SUCCESS) //漫游
		break;//漫游
	}
	delay_ms(100);
	Get_Csq(CSQ_BUFFER);
	printf("GSM信号质量差\r\n");
    }
    sendCommand("AT+QICSGP=1\r\n", "OK", 3000, 2);//设置成GPRS联网模式
    while(!GET_TCP()) //等待TCP连接成功
    {
	times++;
	printf("GPRS正在重新连接,第%d次\r\n",times);
	if( times >=4) return ERROR;
    }
    res = GNSS_INIT();  //GSM初始化成功才会初始化GNSS
    
    GSM_typedef.GSM_Rx_Count =0;
    memset(GSM_typedef.GSM_Rx_Data,0,100); 
  //  MC20LOW_POWER_INIT(); 
    GSM_typedef.MC20E_Ready =1; //可以接受服务器数据（发送数据的时候，就必须为0）

    if(res == SUCCESS) return SUCCESS;
    else return ERROR;
    
    
}

//初始化PWERKEY的IO口
 void POWER_MC20_IO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// GPIOB时钟
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);  //初始化不拉低   
}

//MC20 电源控制
//ON  开机  OFF  关机 GSM_RESET 复位
//返回值：SUCCESS，成功 FAIL失败
_Bool POWER_MC20(u8 MODE)
{
    u8 time=0;
    if(MODE == ON) //开机
    {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//拉低
	delay_ms(1800);
	delay_ms(200);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    }
    if(MODE == OFF)
    {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//拉低
	delay_ms(1000);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    }
    if(MODE == GSM_RESET)
    {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//拉低1s
	delay_ms(1000);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	delay_ms(1800); delay_ms(1800);//至少等待5s
	delay_ms(1800);delay_ms(1800);
	delay_ms(1800);delay_ms(1800);
	printf("MC20 RESETING\r\n");
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//拉低2s
	delay_ms(1800);
	delay_ms(200);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	while(1)
	{
	    if (sendCommand("AT\r\n", "OK\r\n", 3000, 10) == SUCCESS) { printf("EC20 RESET OK\r\n"); MC20_INIT();return SUCCESS;}
	    delay_ms(50);
	    time++;
	    if(time ==4 ) 
	    {
		printf("模块重启失败\r\n");
		return ERROR;
	    }
	}
    }
    return SUCCESS;
}

//发送AT指令 与GSM相关的指令
//timeout为等待命令时间
unsigned int sendCommand(char *Command, char *Response, unsigned long Timeout, unsigned char Retry)
{
    unsigned char n;
    
    USART_MC20E_CLR_Buf();
    for (n = 0; n < Retry; n++)
    {
	u3_printf(Command); 		//发送指令
	printf("\r\n****send****\r\n");
	printf(Command);
	Time_Cont = 0;
	while (Time_Cont < Timeout)
	{
	    delay_ms(100);
	    Time_Cont += 100;
	    if ( strstr(MC20E_Rx_Buff, Response) != NULL )  //判断模块回复的消息是不是期望值  
	    {	
    		
		printf("\r\n****receive****\r\n");
		printf(MC20E_Rx_Buff);
		USART_MC20E_CLR_Buf();
		return SUCCESS;
	    }
           //USART_MC20E_CLR_Buf();
	}
	Time_Cont = 0;
    }
    printf("\r\n***************receive****************\r\n");
    printf(MC20E_Rx_Buff);
    USART_MC20E_CLR_Buf();
    return ERROR;
}
//发送GNSS位置查询指令（没有吧buffer清空）
unsigned int send_GPS_Command(char *Command, char *Response, unsigned long Timeout, unsigned char Retry)
{
    unsigned char n;
    
    USART_MC20E_CLR_Buf();
    GSM_typedef.MC20E_Ready = 0;   //服务器接收关闭
    for (n = 0; n < Retry; n++)
    {
	u3_printf(Command); 		//发送指令
	printf("\r\n****send****\r\n");
	printf(Command);
	Time_Cont = 0;
	while (Time_Cont < Timeout)
	{
	    delay_ms(200);
	    Time_Cont += 100;
	    if ( strstr(MC20E_Rx_Buff, Response) != NULL )  //判断模块回复的消息是不是期望值  
	    {	
    		
		printf("\r\n****receive****\r\n");
		printf(MC20E_Rx_Buff);
                GSM_typedef.MC20E_Ready = 1;   //服务器接收打开
		return SUCCESS;
	    }
            USART_MC20E_CLR_Buf();
	}
	Time_Cont = 0;
    }
    printf("\r\n***************receive****************\r\n");
    printf(MC20E_Rx_Buff);
    USART_MC20E_CLR_Buf();
    GSM_typedef.MC20E_Ready = 1;   //服务器接收关闭
    return ERROR;
}

//连接TCP服务器
//参数：TCPServer；IP地址 Port ：端口号
//返回：SUCCESS，成功 FAIL失败
//测试通过
static _Bool GET_TCP(void)//连接TCPIP/
{
    uint8_t i=0;
    char send_buf[100] = {0};
    memset(send_buf, 0, 100);    //
    strcpy(send_buf, "AT+QIOPEN=\"TCP\",\"");  //发送ip
    strcat(send_buf, TCPServer);
    strcat(send_buf, "\",\"");
    strcat(send_buf, Port);
    strcat(send_buf, "\"\r\n");
    for(i=0;i<3;i++)
    {
      if(sendCommand(send_buf, "CONNECT OK", 10000, 1) == SUCCESS) return SUCCESS;   //可以多等待几次连接TCP
      if(sendCommand(send_buf, "ALREAD", 10000, 1) == SUCCESS) return SUCCESS; 
    }
	return ERROR;
}
//重新连接TCP服务器
static _Bool RESET_GET_CONNECT(void)
{
    u8 res;
     res = GET_TCP(); //重新连接
     if(res == SUCCESS) return SUCCESS;
     if(res == ERROR)
     {
        if(POWER_MC20(GSM_RESET) == SUCCESS) 
        {
          if( MC20_INIT() == SUCCESS)  return SUCCESS;  //重新成功连接网络
          else
            return ERROR;
        }
        else 
        {
          return ERROR;
        }
     }
      return ERROR;
   
}
//gprs发送数据 
//参数 data（发送的数据，data必须为字符串数组）
//返回值：成功SUCCESS 失败ERROR  （当返回失败，可以多次再尝试发送）
_Bool GPRS_Send_Data(char *data)
{
//  static  char send_buf[2]={0};
//    send_buf[0] = 0x1a;  //数据发送指令
//    send_buf[1] = '\0';
//    
//    u8 res=0;
//	//if(sendCommand("AT+QISEND\r\n", ">", 3000, 2) == SUCCESS)
//
////	printf("与MC20模块通信失败\r\n");
////	errorLog("数据发送失败\r\n",GSM_RESET);
////	return ERROR;
//    u3_printf("AT+QISEND\r\n"); 
//    delay_ms(100);
//    u3_printf(data); //直接发送数据（不用判断模块回复的消息）
//    
//    if (sendCommand(send_buf, "OK", 2000, 1) == SUCCESS) 
//    {
//        GPRS_ERROR_NUMBER=0;
//        return SUCCESS;
//    }
//  //  if (sendCommand(send_buf, "K", 3000, 3) == SUCCESS) return SUCCESS;
//    if (sendCommand(send_buf, "ERROR", 2000,1) == SUCCESS)
//    {
//      	    printf("GPRS断线，正在重连\r\n");
//	    res = GET_TCP(); //重新连接
//	    if(res == ERROR)
//	    {
//		if(POWER_MC20(GSM_RESET) == SUCCESS) 
//		{
//		    if( MC20_INIT() == SUCCESS)  return ERROR;  //重新成功连接网络
//		    else
//			return ERROR;
//		}
//		else 
//		{
//		    return ERROR;
//		}
//	    }
//	    else //重新成功连接网络 
//	    {
//		GPRS_ERROR_NUMBER++;
//		if(GPRS_ERROR_NUMBER >=3) //重连成功但是还是不能发送数据
//		{
//		    GPRS_ERROR_NUMBER=0;
//		    POWER_MC20(GSM_RESET);
//		}
//		return ERROR;  
//	    }
//    }
//    return ERROR;
  GSM_typedef.MC20E_Ready = 0;   //服务器接收关闭
    
  char  send_buf[2]={0};
        send_buf[0] = 0x1a;  //数据发送指令
        send_buf[1] = '\0';
  u8    res=0;
  static u8 number_error ;
  
   strcat(data,send_buf);
   //正常成功发送
   if(sendCommand("AT+QISEND\r\n", ">", 4000, 3) == SUCCESS)
   {    
      // sendCommand(data, data, 3000, 1);//发送
      // delay_ms(10);
      if (sendCommand(data, "OK", 5000, 2) == SUCCESS) 
      {
          GSM_typedef.MC20E_Ready = 1;   //服务器接收使能
          return SUCCESS;
      }
      else if(sendCommand(data, "SEND FAIL", 5000, 2) == SUCCESS)
      {
          printf("GPRS断线，正在重连\r\n");
          
          res = GET_TCP(); //重新连接
          if(res == ERROR)
          {
            if(POWER_MC20(GSM_RESET) == SUCCESS) 
            {
                if( MC20_INIT() == SUCCESS) {  GSM_typedef.MC20E_Ready = 1; return ERROR; } //重新成功连接网络
              else
              {
                return ERROR;
              }
            }
            else 
            {
              return ERROR;
            }
         }
        
      }
      
      return ERROR;
   }
   //tcpip连接存在，发送失败
   else if(sendCommand("AT+QISEND\r\n", "RR", 4000, 2) == SUCCESS)   
   {
          if(RESET_GET_CONNECT() == SUCCESS)
          {
            if(sendCommand("AT+QISEND\r\n", ">", 4000, 3) == SUCCESS)
            { 
                if (sendCommand(data, "OK", 5000, 2) == SUCCESS) 
                {
                    GSM_typedef.MC20E_Ready = 1;   //服务器接收使能
                    return SUCCESS;
                }
            }
            GSM_typedef.MC20E_Ready = 1;   //服务器接收使能
            return ERROR;
          }
          else
              return ERROR;
   }
   //模块不响应
   else 
   {  
       if (sendCommand(data, "OK", 5000, 2) == SUCCESS)   { number_error=0; GSM_typedef.MC20E_Ready = 1;return SUCCESS;}
       if (sendCommand(data, "ERROR", 5000, 2) == SUCCESS){ number_error=0; MC20_INIT() ; return ERROR;}
       number_error++;
       if(number_error >=3)
       {         
             number_error=0;
             printf("QISEND失败，正在重连\r\n");
              
             if(RESET_GET_CONNECT() == SUCCESS) {GSM_typedef.MC20E_Ready = 1;   return ERROR; } //重新成功连接网络,但发送数据失败
       }
      return ERROR; 
   }
}

//对错误信息的处理
//参数：eerow输出的错误信息 reset_symble复位标志
//返回值：无
static void errorLog(char *eerow,u8 reset_symble)
{
    if(reset_symble == GSM_EEROW ) //不需要重启 ，认为重启也无法解决问题
    {
	while(1)
	{
	    printf(eerow);
	    delay_ms(1000);
	}
    }
    if(reset_symble == GSM_RESET)
    {
	printf(eerow);
	POWER_MC20(GSM_RESET);
    } 
}



//获取GSM信号质量（满信号-51db时候CSQ的值为31）
void Get_Csq(char *buffer)
{
    char CSQ_BUF[20] = {0};
    char *first;
    char *next;   
    send_GPS_Command("AT+CSQ\r\n", "OK", 3000, 2);//发送登陆GSM网码
    memcpy(CSQ_BUF,MC20E_Rx_Buff,MC20E_Rx_Count);  //复制接收到的信息
    
    USART_MC20E_CLR_Buf(); //清除
    memset(buffer, 0, 2); 
    
    first = strstr(CSQ_BUF,":");
    next  = strstr(CSQ_BUF,",");
    
    if(first !=NULL && next !=NULL)
    {
	first +=2;
	memcpy(buffer,first, next - first);   //CSQ_BUFFER为目标首地址 ，first为被COPY的首地址
	printf("CSQ:%s\r\n",buffer);
    }
    
}

//gnss初始化
//参数：无
//输出：success 成功 error 失败
_Bool GNSS_INIT(void)
{
    if (sendCommand("AT+QGNSSC?\r\n", "+QGNSSC: 1", 5000, 1) == SUCCESS);
    else if (sendCommand("AT+QGNSSC=1\r\n", "OK", 5000, 5) == SUCCESS);//打开GNSS
    else 
    {
	printf("GNSS初始化失败,请检查模块\r\n");
	return ERROR;
    }
    if (sendCommand("AT+QGNSSTS?\r\n", "1", 5000, 2) == SUCCESS);  //时间同步
    else
    {
	printf("GNSS时间同步失败\r\n");
	return ERROR;
    }
    sendCommand("AT+QGNSSEPO=1\r\n", "OK", 5000, 2); //打开EPO，以便加速定位
    sendCommand("AT+QGEPOAID\r\n", "OK", 5000, 2);
    clrStruct();
    return SUCCESS;
}
//清空GNSS结构体数据
//参数：Save_Data
//返回：无
static void clrStruct(void)
{
    Save_Data.isGetData = false; 
    Save_Data.isParseData = false;
    Save_Data.isUsefull = false;
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空GNSS数据BUFFER
    memset(Save_Data.UTCTime, 0, UTCTime_Length);
    memset(Save_Data.latitude, 0, latitude_Length);
    memset(Save_Data.N_S, 0, N_S_Length);
    memset(Save_Data.longitude, 0, longitude_Length);
    memset(Save_Data.E_W, 0, E_W_Length);
}
//解析得到的位置信息
//参数：无
//返回：SUCCESS 成功 ERROR 失败（GNSS信号弱，获取位置不成功）
_Bool parseGpsBuffer(void)
{
    char *subString;
    char *subStringNext;
    char i = 0;
    if (Save_Data.isGetData)
    {
	Save_Data.isGetData = false;
	//printf("**************\r\n");
	//printf(Save_Data.GPS_Buffer);
	for (i = 0 ; i <= 6 ; i++)
	{
	    if (i == 0)
	    {
		if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
		{
		    printf("解析错误\r\n");
		    return ERROR;
		}
	    }
	    else
	    {
		subString++;
		if ((subStringNext = strstr(subString, ",")) != NULL)
		{
		    //char usefullBuffer[2]; 
		    switch(i)
		    {
		     case 1:/*memcpy(Save_Data.UTCTime, subString, subStringNext - subString);*/break;	//获取UTC时间
		     case 2:/*memcpy(usefullBuffer, subString, subStringNext - subString);*/break;	//获取UTC时间
		     case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取维度信息
		     case 4:/*memcpy(Save_Data.N_S, subString, subStringNext - subString);*/break;	//获取N/S
		     case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);Save_Data.isUsefull = true;break;	//获取进度信息
		     case 6:/*memcpy(Save_Data.E_W, subString, subStringNext - subString);*/break;	//获取E/W
		     default:break;
		    }
		    subString = subStringNext;
		    Save_Data.isParseData = true;
//		    if(usefullBuffer[0] == 'A')
//			Save_Data.isUsefull = true;
//		    else if(usefullBuffer[0] == 'V')
//			Save_Data.isUsefull = false;
		}
		else
		{
		    printf("GPS信号弱\r\n");
		    return ERROR;
		}
	    }  
	}
	return SUCCESS;
    }
    else
    {
	printf("解析错误\r\n");
    }
    return ERROR;
}

//实时发送位置信息
//参数：无
//返回：SUCCESS 成功 ERROR 失败（GNSS信号弱，获取位置不成功）
void GET_LOCATION(void)
{
     
    if (send_GPS_Command("AT+QGNSSRD=\"NMEA/RMC\"\r\n", "OK", 3000, 10) == SUCCESS);  //获取位置指令
    else 
    {
	printf("位置获取失败，请移动到开阔位置\r\n");
	//GNSS_INIT();
        return ;
    }
    Save_Data.isGetData = true;
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空BUFFER
    memcpy(Save_Data.GPS_Buffer, MC20E_Rx_Buff,strlen(MC20E_Rx_Buff));  //复制接收到的信息
    USART_MC20E_CLR_Buf();
    /******************发送GPS位置信息*********************/
    parseGpsBuffer();  //解析位置信息
    if (Save_Data.isParseData)
    {
	Save_Data.isParseData = false;
	//    u2_printf("Save_Data.UTCTime = ");
	//    u2_printf(Save_Data.UTCTime);
	//    u2_printf("\r\n");
	
	if(Save_Data.isUsefull)
	{
	   Save_Data.isUsefull = false;
           sprintf(pMulti_Senser->GPS_Data_BUF,"N:%sE:%s",Save_Data.latitude,Save_Data.longitude);
           clrStruct();  
//	    strcat(GPRS_Send_Buff,"N ");
//	    strcat(GPRS_Send_Buff,Save_Data.latitude);
//	    strcat(GPRS_Send_Buff,"E ");
//	    strcat(GPRS_Send_Buff,Save_Data.longitude);
            
	   // res = GPRS_Send_Data(GPRS_Send_Buff);
	  //  memset(GPRS_Send_Buff, 0, GPRS_Send_Buff_len);  //清空发送BUFFER
//	    if(res == SUCCESS)
//	    {
//		return SUCCESS;
//	    }
//	    else 
//		return ERROR;
	}
	else
	{
	    printf("GPS 还没有定位成功!\r\n");
	    
	}
	
    }
}




