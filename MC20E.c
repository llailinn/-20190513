/**
******************************************************************************
* @file    MC20E.c 
* @author  Application Team
* @version V1.0.0
* @date    2019-4-18
* @brief   MC20E���ɺ���
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
extern GSM_TypeDef GSM_typedef;    //GSM���շ��������ݴ�Žṹ��
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
_SaveData Save_Data;//GNSS�ṹ��
char TCPServer[] = "103.46.128.49";		//TCP���ӵ�IP��ַ120.76.205.151
char Port[] = "45669";	     //30333
char CSQ_BUFFER[2]={0};
u8 GPRS_ERROR_NUMBER =0;  //GPRS�����Ĵ���������TCP�ɹ����������ݷ���ʧ�ܣ�
char GPRS_Send_Buff[20]={0};  //gprs����BUFFRT
unsigned long  Time_Cont = 0;       //��ʱ��������
unsigned int   count = 0;


//MC20�͹��ĳ�ʼ��
//��������
//���أ�
static void MC20LOW_POWER_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	// GPIOBʱ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  PBout(9) = 0; //�Ȳ�����˯��ģʽ
  
 sendCommand("AT+QSCLK=1\r\n", "OK", 3000, 5) ;//��ȴ�30s

}

//����͹���ģʽ
//��������
//���أ���
void MC20_INLOW_POWER(void)
{
  PBout(9) = 1;
  delay_ms(20);
}
//�˳��͹���ģʽ
//��������
//���أ���
void MC20_OUTLOW_POWER(void)
{
  PBout(9) = 0;
  delay_ms(20);
}


//��ʼ��MC20
//��������
//���أ�SUCCESS���ɹ� FAILʧ��
//����ͨ��
_Bool MC20_INIT(void)
{
    u8 times=0;
    u8 res;
    u3_printf("AT\r\n"); 
    delay_ms(100);//�ȴ�������ͬ��
    while(sendCommand("AT\r\n", "OK", 3000, 10) != SUCCESS)  //��ȴ�30s
    {
	errorLog("���ģ��ʧ��\r\n",GSM_RESET);
	delay_ms(100);
    }
    sendCommand("ATE0\r\n", "OK", 3000, 5);
    while(sendCommand("AT+IPR=115200\r\n", "OK", 3000, 10) != SUCCESS)//�̶�������
    {
	errorLog("AT+IPR�����ʹ̶�ʧ��\r\n",GSM_RESET);
	delay_ms(100);
    }
    while(sendCommand("AT+CPIN?\r\n", "READY", 3000, 10) != SUCCESS)  //����SIM��
    { 
	errorLog("����SIM��\r\n",GSM_RESET);
	delay_ms(100);
    }
    sendCommand("AT+CREG=1\r\n", "OK", 3000, 2);//���͵�½GSM����
    sendCommand("AT+CREG=2\r\n", "OK", 3000, 2);
    sendCommand("AT+QVBATT=0,3452\r\n", "OK", 3000, 2);
    
    while(1)   //ע��GSM��
    {
	if (sendCommand("AT+CREG?\r\n", "1", 7000, 8) == SUCCESS) 
	{
	    if (sendCommand("AT+CGREG?\r\n", "1", 6000, 8) == SUCCESS) //������
		break;//������ ��ȵ�50S
	}
	if(sendCommand("AT+CREG?\r\n", "5", 5000, 1) == SUCCESS)  
	{
	    if(sendCommand("AT+CGREG?\r\n", "5", 5000, 1) == SUCCESS) //����
		break;//����
	}
	delay_ms(100);
	Get_Csq(CSQ_BUFFER);
	printf("GSM�ź�������\r\n");
    }
    sendCommand("AT+QICSGP=1\r\n", "OK", 3000, 2);//���ó�GPRS����ģʽ
    while(!GET_TCP()) //�ȴ�TCP���ӳɹ�
    {
	times++;
	printf("GPRS������������,��%d��\r\n",times);
	if( times >=4) return ERROR;
    }
    res = GNSS_INIT();  //GSM��ʼ���ɹ��Ż��ʼ��GNSS
    
    GSM_typedef.GSM_Rx_Count =0;
    memset(GSM_typedef.GSM_Rx_Data,0,100); 
  //  MC20LOW_POWER_INIT(); 
    GSM_typedef.MC20E_Ready =1; //���Խ��ܷ��������ݣ��������ݵ�ʱ�򣬾ͱ���Ϊ0��

    if(res == SUCCESS) return SUCCESS;
    else return ERROR;
    
    
}

//��ʼ��PWERKEY��IO��
 void POWER_MC20_IO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// GPIOBʱ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);  //��ʼ��������   
}

//MC20 ��Դ����
//ON  ����  OFF  �ػ� GSM_RESET ��λ
//����ֵ��SUCCESS���ɹ� FAILʧ��
_Bool POWER_MC20(u8 MODE)
{
    u8 time=0;
    if(MODE == ON) //����
    {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//����
	delay_ms(1800);
	delay_ms(200);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    }
    if(MODE == OFF)
    {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//����
	delay_ms(1000);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    }
    if(MODE == GSM_RESET)
    {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//����1s
	delay_ms(1000);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	delay_ms(1800); delay_ms(1800);//���ٵȴ�5s
	delay_ms(1800);delay_ms(1800);
	delay_ms(1800);delay_ms(1800);
	printf("MC20 RESETING\r\n");
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//����2s
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
		printf("ģ������ʧ��\r\n");
		return ERROR;
	    }
	}
    }
    return SUCCESS;
}

//����ATָ�� ��GSM��ص�ָ��
//timeoutΪ�ȴ�����ʱ��
unsigned int sendCommand(char *Command, char *Response, unsigned long Timeout, unsigned char Retry)
{
    unsigned char n;
    
    USART_MC20E_CLR_Buf();
    for (n = 0; n < Retry; n++)
    {
	u3_printf(Command); 		//����ָ��
	printf("\r\n****send****\r\n");
	printf(Command);
	Time_Cont = 0;
	while (Time_Cont < Timeout)
	{
	    delay_ms(100);
	    Time_Cont += 100;
	    if ( strstr(MC20E_Rx_Buff, Response) != NULL )  //�ж�ģ��ظ�����Ϣ�ǲ�������ֵ  
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
//����GNSSλ�ò�ѯָ�û�а�buffer��գ�
unsigned int send_GPS_Command(char *Command, char *Response, unsigned long Timeout, unsigned char Retry)
{
    unsigned char n;
    
    USART_MC20E_CLR_Buf();
    GSM_typedef.MC20E_Ready = 0;   //���������չر�
    for (n = 0; n < Retry; n++)
    {
	u3_printf(Command); 		//����ָ��
	printf("\r\n****send****\r\n");
	printf(Command);
	Time_Cont = 0;
	while (Time_Cont < Timeout)
	{
	    delay_ms(200);
	    Time_Cont += 100;
	    if ( strstr(MC20E_Rx_Buff, Response) != NULL )  //�ж�ģ��ظ�����Ϣ�ǲ�������ֵ  
	    {	
    		
		printf("\r\n****receive****\r\n");
		printf(MC20E_Rx_Buff);
                GSM_typedef.MC20E_Ready = 1;   //���������մ�
		return SUCCESS;
	    }
            USART_MC20E_CLR_Buf();
	}
	Time_Cont = 0;
    }
    printf("\r\n***************receive****************\r\n");
    printf(MC20E_Rx_Buff);
    USART_MC20E_CLR_Buf();
    GSM_typedef.MC20E_Ready = 1;   //���������չر�
    return ERROR;
}

//����TCP������
//������TCPServer��IP��ַ Port ���˿ں�
//���أ�SUCCESS���ɹ� FAILʧ��
//����ͨ��
static _Bool GET_TCP(void)//����TCPIP/
{
    uint8_t i=0;
    char send_buf[100] = {0};
    memset(send_buf, 0, 100);    //
    strcpy(send_buf, "AT+QIOPEN=\"TCP\",\"");  //����ip
    strcat(send_buf, TCPServer);
    strcat(send_buf, "\",\"");
    strcat(send_buf, Port);
    strcat(send_buf, "\"\r\n");
    for(i=0;i<3;i++)
    {
      if(sendCommand(send_buf, "CONNECT OK", 10000, 1) == SUCCESS) return SUCCESS;   //���Զ�ȴ���������TCP
      if(sendCommand(send_buf, "ALREAD", 10000, 1) == SUCCESS) return SUCCESS; 
    }
	return ERROR;
}
//��������TCP������
static _Bool RESET_GET_CONNECT(void)
{
    u8 res;
     res = GET_TCP(); //��������
     if(res == SUCCESS) return SUCCESS;
     if(res == ERROR)
     {
        if(POWER_MC20(GSM_RESET) == SUCCESS) 
        {
          if( MC20_INIT() == SUCCESS)  return SUCCESS;  //���³ɹ���������
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
//gprs�������� 
//���� data�����͵����ݣ�data����Ϊ�ַ������飩
//����ֵ���ɹ�SUCCESS ʧ��ERROR  ��������ʧ�ܣ����Զ���ٳ��Է��ͣ�
_Bool GPRS_Send_Data(char *data)
{
//  static  char send_buf[2]={0};
//    send_buf[0] = 0x1a;  //���ݷ���ָ��
//    send_buf[1] = '\0';
//    
//    u8 res=0;
//	//if(sendCommand("AT+QISEND\r\n", ">", 3000, 2) == SUCCESS)
//
////	printf("��MC20ģ��ͨ��ʧ��\r\n");
////	errorLog("���ݷ���ʧ��\r\n",GSM_RESET);
////	return ERROR;
//    u3_printf("AT+QISEND\r\n"); 
//    delay_ms(100);
//    u3_printf(data); //ֱ�ӷ������ݣ������ж�ģ��ظ�����Ϣ��
//    
//    if (sendCommand(send_buf, "OK", 2000, 1) == SUCCESS) 
//    {
//        GPRS_ERROR_NUMBER=0;
//        return SUCCESS;
//    }
//  //  if (sendCommand(send_buf, "K", 3000, 3) == SUCCESS) return SUCCESS;
//    if (sendCommand(send_buf, "ERROR", 2000,1) == SUCCESS)
//    {
//      	    printf("GPRS���ߣ���������\r\n");
//	    res = GET_TCP(); //��������
//	    if(res == ERROR)
//	    {
//		if(POWER_MC20(GSM_RESET) == SUCCESS) 
//		{
//		    if( MC20_INIT() == SUCCESS)  return ERROR;  //���³ɹ���������
//		    else
//			return ERROR;
//		}
//		else 
//		{
//		    return ERROR;
//		}
//	    }
//	    else //���³ɹ��������� 
//	    {
//		GPRS_ERROR_NUMBER++;
//		if(GPRS_ERROR_NUMBER >=3) //�����ɹ����ǻ��ǲ��ܷ�������
//		{
//		    GPRS_ERROR_NUMBER=0;
//		    POWER_MC20(GSM_RESET);
//		}
//		return ERROR;  
//	    }
//    }
//    return ERROR;
  GSM_typedef.MC20E_Ready = 0;   //���������չر�
    
  char  send_buf[2]={0};
        send_buf[0] = 0x1a;  //���ݷ���ָ��
        send_buf[1] = '\0';
  u8    res=0;
  static u8 number_error ;
  
   strcat(data,send_buf);
   //�����ɹ�����
   if(sendCommand("AT+QISEND\r\n", ">", 4000, 3) == SUCCESS)
   {    
      // sendCommand(data, data, 3000, 1);//����
      // delay_ms(10);
      if (sendCommand(data, "OK", 5000, 2) == SUCCESS) 
      {
          GSM_typedef.MC20E_Ready = 1;   //����������ʹ��
          return SUCCESS;
      }
      else if(sendCommand(data, "SEND FAIL", 5000, 2) == SUCCESS)
      {
          printf("GPRS���ߣ���������\r\n");
          
          res = GET_TCP(); //��������
          if(res == ERROR)
          {
            if(POWER_MC20(GSM_RESET) == SUCCESS) 
            {
                if( MC20_INIT() == SUCCESS) {  GSM_typedef.MC20E_Ready = 1; return ERROR; } //���³ɹ���������
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
   //tcpip���Ӵ��ڣ�����ʧ��
   else if(sendCommand("AT+QISEND\r\n", "RR", 4000, 2) == SUCCESS)   
   {
          if(RESET_GET_CONNECT() == SUCCESS)
          {
            if(sendCommand("AT+QISEND\r\n", ">", 4000, 3) == SUCCESS)
            { 
                if (sendCommand(data, "OK", 5000, 2) == SUCCESS) 
                {
                    GSM_typedef.MC20E_Ready = 1;   //����������ʹ��
                    return SUCCESS;
                }
            }
            GSM_typedef.MC20E_Ready = 1;   //����������ʹ��
            return ERROR;
          }
          else
              return ERROR;
   }
   //ģ�鲻��Ӧ
   else 
   {  
       if (sendCommand(data, "OK", 5000, 2) == SUCCESS)   { number_error=0; GSM_typedef.MC20E_Ready = 1;return SUCCESS;}
       if (sendCommand(data, "ERROR", 5000, 2) == SUCCESS){ number_error=0; MC20_INIT() ; return ERROR;}
       number_error++;
       if(number_error >=3)
       {         
             number_error=0;
             printf("QISENDʧ�ܣ���������\r\n");
              
             if(RESET_GET_CONNECT() == SUCCESS) {GSM_typedef.MC20E_Ready = 1;   return ERROR; } //���³ɹ���������,����������ʧ��
       }
      return ERROR; 
   }
}

//�Դ�����Ϣ�Ĵ���
//������eerow����Ĵ�����Ϣ reset_symble��λ��־
//����ֵ����
static void errorLog(char *eerow,u8 reset_symble)
{
    if(reset_symble == GSM_EEROW ) //����Ҫ���� ����Ϊ����Ҳ�޷��������
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



//��ȡGSM�ź����������ź�-51dbʱ��CSQ��ֵΪ31��
void Get_Csq(char *buffer)
{
    char CSQ_BUF[20] = {0};
    char *first;
    char *next;   
    send_GPS_Command("AT+CSQ\r\n", "OK", 3000, 2);//���͵�½GSM����
    memcpy(CSQ_BUF,MC20E_Rx_Buff,MC20E_Rx_Count);  //���ƽ��յ�����Ϣ
    
    USART_MC20E_CLR_Buf(); //���
    memset(buffer, 0, 2); 
    
    first = strstr(CSQ_BUF,":");
    next  = strstr(CSQ_BUF,",");
    
    if(first !=NULL && next !=NULL)
    {
	first +=2;
	memcpy(buffer,first, next - first);   //CSQ_BUFFERΪĿ���׵�ַ ��firstΪ��COPY���׵�ַ
	printf("CSQ:%s\r\n",buffer);
    }
    
}

//gnss��ʼ��
//��������
//�����success �ɹ� error ʧ��
_Bool GNSS_INIT(void)
{
    if (sendCommand("AT+QGNSSC?\r\n", "+QGNSSC: 1", 5000, 1) == SUCCESS);
    else if (sendCommand("AT+QGNSSC=1\r\n", "OK", 5000, 5) == SUCCESS);//��GNSS
    else 
    {
	printf("GNSS��ʼ��ʧ��,����ģ��\r\n");
	return ERROR;
    }
    if (sendCommand("AT+QGNSSTS?\r\n", "1", 5000, 2) == SUCCESS);  //ʱ��ͬ��
    else
    {
	printf("GNSSʱ��ͬ��ʧ��\r\n");
	return ERROR;
    }
    sendCommand("AT+QGNSSEPO=1\r\n", "OK", 5000, 2); //��EPO���Ա���ٶ�λ
    sendCommand("AT+QGEPOAID\r\n", "OK", 5000, 2);
    clrStruct();
    return SUCCESS;
}
//���GNSS�ṹ������
//������Save_Data
//���أ���
static void clrStruct(void)
{
    Save_Data.isGetData = false; 
    Save_Data.isParseData = false;
    Save_Data.isUsefull = false;
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //���GNSS����BUFFER
    memset(Save_Data.UTCTime, 0, UTCTime_Length);
    memset(Save_Data.latitude, 0, latitude_Length);
    memset(Save_Data.N_S, 0, N_S_Length);
    memset(Save_Data.longitude, 0, longitude_Length);
    memset(Save_Data.E_W, 0, E_W_Length);
}
//�����õ���λ����Ϣ
//��������
//���أ�SUCCESS �ɹ� ERROR ʧ�ܣ�GNSS�ź�������ȡλ�ò��ɹ���
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
		    printf("��������\r\n");
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
		     case 1:/*memcpy(Save_Data.UTCTime, subString, subStringNext - subString);*/break;	//��ȡUTCʱ��
		     case 2:/*memcpy(usefullBuffer, subString, subStringNext - subString);*/break;	//��ȡUTCʱ��
		     case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//��ȡά����Ϣ
		     case 4:/*memcpy(Save_Data.N_S, subString, subStringNext - subString);*/break;	//��ȡN/S
		     case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);Save_Data.isUsefull = true;break;	//��ȡ������Ϣ
		     case 6:/*memcpy(Save_Data.E_W, subString, subStringNext - subString);*/break;	//��ȡE/W
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
		    printf("GPS�ź���\r\n");
		    return ERROR;
		}
	    }  
	}
	return SUCCESS;
    }
    else
    {
	printf("��������\r\n");
    }
    return ERROR;
}

//ʵʱ����λ����Ϣ
//��������
//���أ�SUCCESS �ɹ� ERROR ʧ�ܣ�GNSS�ź�������ȡλ�ò��ɹ���
void GET_LOCATION(void)
{
     
    if (send_GPS_Command("AT+QGNSSRD=\"NMEA/RMC\"\r\n", "OK", 3000, 10) == SUCCESS);  //��ȡλ��ָ��
    else 
    {
	printf("λ�û�ȡʧ�ܣ����ƶ�������λ��\r\n");
	//GNSS_INIT();
        return ;
    }
    Save_Data.isGetData = true;
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //���BUFFER
    memcpy(Save_Data.GPS_Buffer, MC20E_Rx_Buff,strlen(MC20E_Rx_Buff));  //���ƽ��յ�����Ϣ
    USART_MC20E_CLR_Buf();
    /******************����GPSλ����Ϣ*********************/
    parseGpsBuffer();  //����λ����Ϣ
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
	  //  memset(GPRS_Send_Buff, 0, GPRS_Send_Buff_len);  //��շ���BUFFER
//	    if(res == SUCCESS)
//	    {
//		return SUCCESS;
//	    }
//	    else 
//		return ERROR;
	}
	else
	{
	    printf("GPS ��û�ж�λ�ɹ�!\r\n");
	    
	}
	
    }
}




