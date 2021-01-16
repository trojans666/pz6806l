#include <string.h>

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "bsp_usart.h"

/*!< USART2(RS485) PA2(TX) PA3(RX) */
#define USART2_TX_APB          RCC_APB2Periph_GPIOA
#define USART2_TX_PORT         GPIOA
#define USART2_TX_PIN          GPIO_Pin_2

#define USART2_RX_APB          RCC_APB2Periph_GPIOA
#define USART2_RX_PORT         GPIOA
#define USART2_RX_PIN          GPIO_Pin_3

/*!< USART1(RS232) PA9(TX) PA10(RX) */
#define USART1_TX_APB          RCC_APB2Periph_GPIOA
#define USART1_TX_PORT         GPIOA
#define USART1_TX_PIN          GPIO_Pin_9

#define USART1_RX_APB          RCC_APB2Periph_GPIOA
#define USART1_RX_PORT         GPIOA
#define USART1_RX_PIN          GPIO_Pin_10


/*!< USART3(RS232) PB10(TX) PB11(RX) */
#define USART3_TX_APB          RCC_APB2Periph_GPIOB
#define USART3_TX_PORT         GPIOB
#define USART3_TX_PIN          GPIO_Pin_10

#define USART3_RX_APB          RCC_APB2Periph_GPIOB
#define USART3_RX_PORT         GPIOB
#define USART3_RX_PIN          GPIO_Pin_11



static uint8_t Usart1SendBuffer[USART1_SEND_BUF_LEN];
static uint8_t Usart1RecvBuffer[USART1_RECV_BUF_LEN];
UsartBuf_Struct pUsart1;


static uint8_t Usart2SendBuffer[USART2_SEND_BUF_LEN];
static uint8_t Usart2RecvBuffer[USART2_RECV_BUF_LEN];
UsartBuf_Struct pUsart2;


static uint8_t Usart3SendBuffer[USART3_SEND_BUF_LEN];
static uint8_t Usart3RecvBuffer[USART3_RECV_BUF_LEN];
UsartBuf_Struct pUsart3;

struct usart_priv
{
  USART_TypeDef *u;
  UsartBuf_Struct *p;
};

static struct usart_priv mUsart[USART_COM_MAX] =
{
  {USART1,&pUsart1},
  {USART2,&pUsart2},
  {USART3,&pUsart3}
};

static void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(USART1_TX_APB | USART1_RX_APB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;  //PA.9,TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);

    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);

    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET) {;}
    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //使能接收中断
}

static void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(USART2_TX_APB | USART2_RX_APB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE );

    GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USART2_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USART2_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART2_TX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);

    while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET) {;}
    USART_ClearFlag(USART2, USART_FLAG_TC);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //使能接收中断
}

static void USART3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(USART3_TX_APB | USART3_RX_APB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE );

    GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USART3_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART3_TX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_Cmd(USART3, ENABLE);

    while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET) {;}
    USART_ClearFlag(USART3, USART_FLAG_TC);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  //使能接收中断
}

static void usart_buffer_init(void)
{
    UsartBuf_Struct *pBuf;

    /* USART1 */
    pBuf = &pUsart1;
    pBuf->SendBuffer = Usart1SendBuffer;
    pBuf->SendBufferLen = USART1_SEND_BUF_LEN;
    pBuf->SendBytes = 0;
    pBuf->SendPos = 0;
    (void)memset(pBuf->SendBuffer,0x00,pBuf->SendBufferLen);

    pBuf->RecvBuffer = Usart1RecvBuffer;
    pBuf->RecvBufferLen = USART1_RECV_BUF_LEN;
    pBuf->RecvBytes = 0;
    pBuf->RecvPos = 0;
    (void)memset(pBuf->RecvBuffer,0x00,pBuf->RecvBufferLen);

    pBuf->rxBusy = 0;
    pBuf->txBusy = 0;
    pBuf->rxOverTime = USART_RXOVER_TIME;
    pBuf->txErrTime = USART_TXERR_TIME;

    /* USART2 */
    pBuf = &pUsart2;
    pBuf->SendBuffer = Usart2SendBuffer;
    pBuf->SendBufferLen = USART2_SEND_BUF_LEN;
    pBuf->SendBytes = 0;
    pBuf->SendPos = 0;
    (void)memset(pBuf->SendBuffer,0x00,pBuf->SendBufferLen);

    pBuf->RecvBuffer = Usart2RecvBuffer;
    pBuf->RecvBufferLen = USART2_RECV_BUF_LEN;
    pBuf->RecvBytes = 0;
    pBuf->RecvPos = 0;
    (void)memset(pBuf->RecvBuffer,0x00,pBuf->RecvBufferLen);

    pBuf->rxBusy = 0;
    pBuf->txBusy = 0;
    pBuf->rxOverTime = USART_RXOVER_TIME;
    pBuf->txErrTime = USART_TXERR_TIME;

    /* USART3 */
    pBuf = &pUsart3;
    pBuf->SendBuffer = Usart3SendBuffer;
    pBuf->SendBufferLen = USART3_SEND_BUF_LEN;
    pBuf->SendBytes = 0;
    pBuf->SendPos = 0;
    (void)memset(pBuf->SendBuffer,0x00,pBuf->SendBufferLen);

    pBuf->RecvBuffer = Usart3RecvBuffer;
    pBuf->RecvBufferLen = USART3_RECV_BUF_LEN;
    pBuf->RecvBytes = 0;
    pBuf->RecvPos = 0;
    (void)memset(pBuf->RecvBuffer,0x00,pBuf->RecvBufferLen);

    pBuf->rxBusy = 0;
    pBuf->txBusy = 0;
    pBuf->rxOverTime = USART_RXOVER_TIME;
    pBuf->txErrTime = USART_TXERR_TIME;
}

void bsp_usart_init(void)
{
    USART_DeInit(USART1);
    USART1_Init();

    USART_DeInit(USART2);
    USART2_Init();

    USART_DeInit(USART3);
    USART3_Init();

    usart_buffer_init();
}

UsartBuf_Struct *bsp_get_usart(UsartCom com)
{
    UsartBuf_Struct *p = (void *)0;
    /* 如果这里严谨点 要判断或者 assert */
    p = mUsart[com].p;
    return p;
}

/* 定时器中断中使用 */
void usart_rx_over_check(void)
{
    uint8_t pos = 0;
    for(pos = 0;pos < USART_COM_MAX;pos++)
    {
        if(mUsart[pos].p->rxOverTime > 0)
        {
            mUsart[pos].p->rxOverTime--;
        }
        if(mUsart[pos].p->txBusy)
        {
            if(mUsart[pos].p->txErrTime-- == 0)
                mUsart[pos].p->rxBusy = 1;
        }
    }
}

static void usart_irq(UsartCom com)
{
    unsigned char dat;
    if(USART_GetITStatus(mUsart[com].u,USART_IT_RXNE) != RESET)
    {
        /*接收中断 */
        dat = USART_ReceiveData(mUsart[com].u);
        if(mUsart[com].p->RecvPos < mUsart[com].p->RecvBufferLen)
        {
            mUsart[com].p->RecvBuffer[mUsart[com].p->RecvPos] = dat;
            mUsart[com].p->RecvPos++;
            mUsart[com].p->RecvBytes++;
        }
        mUsart[com].p->rxOverTime = USART_RXOVER_TIME;
        USART_ClearFlag(mUsart[com].u,USART_FLAG_RXNE);
    }
    if(USART_GetFlagStatus(mUsart[com].u,USART_FLAG_ORE) != RESET)
    {
        dat = USART_ReceiveData(mUsart[com].u);
        USART_ClearFlag(mUsart[com].u,USART_FLAG_ORE);
    }

    if(USART_GetITStatus(mUsart[com].u,USART_IT_TXE) != RESET)
    {
        /* 发送中断 */
        if(mUsart[com].p->SendPos < mUsart[com].p->SendBytes)
        {
            mUsart[com].u->DR = mUsart[com].p->SendBuffer[mUsart[com].p->SendPos];
            mUsart[com].p->SendPos++;

            mUsart[com].p->txErrTime = USART_TXERR_TIME;
        }
        else
        {
            mUsart[com].p->txErrTime = 0;
            mUsart[com].p->txBusy = 0;
						mUsart[com].p->SendPos = 0;
						mUsart[com].p->SendBytes = 0;
					
            /* 收发切换(rs485) switch to read mode */
						
						USART_ITConfig(mUsart[com].u,USART_IT_TXE,DISABLE);
						USART_ITConfig(mUsart[com].u,USART_IT_TC,ENABLE);
        }
        USART_ClearFlag(mUsart[com].u,USART_FLAG_TXE);
    }
		
		if(USART_GetITStatus(mUsart[com].u,USART_IT_TC) != RESET)
		{
				USART_ITConfig(mUsart[com].u,USART_IT_TC,DISABLE);
				USART_ClearFlag(mUsart[com].u,USART_FLAG_TC);
		}
}

void USART1_IRQHandler(void)
{
		usart_irq(USART_COM1);
}

void USART2_IRQHandler(void)
{
		usart_irq(USART_COM2);
}

void USART3_IRQHandler(void)
{
		usart_irq(USART_COM3);
}

void usart_writedata(UsartCom iNum,uint8_t *pFrame,uint16_t iLength)
{
		uint8_t *dat = pFrame;
		uint16_t len = iLength;
	
		/* iLength < mUsart[iNum].p->SendBufferLen; */
	
		if(mUsart[iNum].p->rxBusy)
		{
				mUsart[iNum].p->txBusy = 0;
				mUsart[iNum].p->txBusy = 0;
		}
		
		while(len--)
		{
				mUsart[iNum].p->SendBuffer[mUsart[iNum].p->SendPos] = *dat;
				mUsart[iNum].p->SendPos++;
				dat++;
		}
		
		if(0 == mUsart[iNum].p->txBusy)
		{
			/* 转换模式为 发送 引脚转换*/
			
			
			mUsart[iNum].p->txBusy = 1;
			mUsart[iNum].p->rxBusy = 0;
			mUsart[iNum].p->txErrTime = USART_TXERR_TIME;
			
			USART_ITConfig(mUsart[iNum].u,USART_IT_TXE,ENABLE);
		}
}

void usart_clear_recvbuf(UsartCom iNum)
{
	  (void)memset(mUsart[iNum].p->RecvBuffer,0x00,mUsart[iNum].p->RecvBufferLen);
		mUsart[iNum].p->RecvBytes = 0;
		mUsart[iNum].p->RecvPos = 0;
}

/* 也可以把函数里的内容复制到中断中执行,这样就免除了拷贝操作 */
void usart_read_data(UsartCom iNum,uint8_t *pFrame,uint16_t iLength)
{
		uint16_t len;
		if(mUsart[iNum].p->rxOverTime == 0)
		{
			len = iLength > mUsart[iNum].p->RecvBytes ? mUsart[iNum].p->RecvBytes : iLength;
			memcpy(pFrame,mUsart[iNum].p->RecvBuffer,len);
			
			usart_clear_recvbuf(iNum);
		}
}
