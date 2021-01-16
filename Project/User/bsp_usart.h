#ifndef __BSP_USART_H
#define __BSP_USART_H

#include <stdint.h>

#define USART1_SEND_BUF_LEN     128
#define USART1_RECV_BUF_LEN     128

#define USART2_SEND_BUF_LEN     128
#define USART2_RECV_BUF_LEN     128

#define USART3_SEND_BUF_LEN     128
#define USART3_RECV_BUF_LEN     128

#define USART_TXERR_TIME        7 /*!< 7ms (MAX 7ms) */
#define USART_RXOVER_TIME       3 /*!< 3ms (MAX 7ms) */

typedef enum
{
    USART_COM1,
    USART_COM2,
    USART_COM3,
    USART_COM_MAX
}UsartCom;


typedef struct
{
    uint8_t *SendBuffer;
    uint16_t SendPos;
    uint16_t SendBytes;
    uint16_t SendBufferLen;

    uint16_t RecvBufferLen;
    uint16_t RecvBytes;
    uint16_t RecvPos;
    uint8_t *RecvBuffer;

    uint8_t txBusy:1;
    uint8_t rxBusy:1;
    uint8_t rxOverTime:3;
    uint8_t txErrTime:3;
}UsartBuf_Struct;

void bsp_usart_init(void);
UsartBuf_Struct *bsp_get_usart(UsartCom com);

void usart_rx_over_check(void);

void usart_writedata(UsartCom iNum,uint8_t *pFrame,uint16_t iLength);
void usart_read_data(UsartCom iNum,uint8_t *pFrame,uint16_t iLength);

#endif
