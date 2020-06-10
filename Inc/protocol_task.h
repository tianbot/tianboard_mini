#ifndef _PROTOCOL_TASK_H_
#define _PROTOCOL_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "protocol.h"

#define PROTOCOL_MSG_LEN 128
#define PROTOCOL_MSG_QUENE_SIZE 8
#define PROTOCOL_TIMEOUT 500

#define UART3_TX_FINISH 0x01

#define CONNECTED 0
#define DISCONNECTED 1

#define RADIAN_COEF 57.29578

#pragma pack(push)
#pragma pack(1)
struct ProtocolMsg {
  uint8_t Msg[PROTOCOL_MSG_LEN];
  uint16_t MsgLen;
};
#pragma pack(pop)

extern struct ProtocolMsg *pProtocolMsg;

extern osMailQId ProtocolRxMail;
extern osMailQId ProtocolTxMail;

extern osThreadId ProtocolSendTaskHandle;

void ProtocolTaskInit(void);
void ProtocolSend(uint16_t pack_type, uint8_t *msg, uint16_t len);
#endif
