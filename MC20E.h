#include "stm32f10x.h"
#include <sys/sys.h>

typedef struct
{    
    bool GSM_Receive_Correct;
    bool MC20E_Ready;    //MC20能接收服务器的数据指令
    char GSM_Rx_Data[GSMTx_MAX_LENGTH];
    char DATA_BUF[15];
    //char GSM_Tx_Data;
    uint8_t GSM_Rx_Count;
   // uint8_t GSM_Tx_Count;
}GSM_TypeDef;
extern char CSQ_BUFFER[];
extern char GPRS_Send_Buff[];
extern _SaveData Save_Data;

void MC20_INLOW_POWER(void);
void MC20_OUTLOW_POWER(void);
_Bool MC20_INIT(void);
_Bool GPRS_Send_Data(char *data);
void GET_LOCATION(void);
void Usart_MC20E_init(u32 bound);
void Get_Csq(char *buffer);
unsigned int sendCommand(char *Command, char *Response, unsigned long Timeout, unsigned char Retry);
 void POWER_MC20_IO(void);
 _Bool POWER_MC20(u8 MODE);