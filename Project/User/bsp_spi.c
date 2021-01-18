#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"

#include "bsp_spi.h"

/* CLK MISO MOSI EN(片选) */
/*!< SPI1 */
#define SPI1_SCK_APB            RCC_APB2Periph_GPIOA
#define SPI1_SCK_PORT           GPIOA
#define SPI1_SCK_PIN            GPIO_Pin_4

#define SPI1_MISO_APB           RCC_APB2Periph_GPIOA
#define SPI1_MISO_PORT          GPIOA
#define SPI1_MISO_PIN           GPIO_Pin_5

#define SPI1_MOSI_APB           RCC_APB2Periph_GPIOA
#define SPI1_MOSI_PORT          GPIOA
#define SPI1_MOSI_PIN           GPIO_Pin_7

/*!< SPI2 */
/* 片选 */
#define SPI2_NSS_APB            RCC_APB2Periph_GPIOB
#define SPI2_NSS_PORT           GPIOB
#define SPI2_NSS_PIN            GPIO_Pin_12

#define SPI2_SCK_APB            RCC_APB2Periph_GPIOB
#define SPI2_SCK_PORT           GPIOB
#define SPI2_SCK_PIN            GPIO_Pin_13

#define SPI2_MISO_APB           RCC_APB2Periph_GPIOB
#define SPI2_MISO_PORT          GPIOB
#define SPI2_MISO_PIN           GPIO_Pin_14

#define SPI2_MOSI_APB           RCC_APB2Periph_GPIOA
#define SPI2_MOSI_PORT          GPIOA
#define SPI2_MOSI_PIN           GPIO_Pin_15

/**
* spi message struct
*/
struct spi_message
{
    uint8_t *send_buf;
    uint8_t *recv_buf;
    uint16_t length;

    uint8_t cs_take:1; //已经被片选
    uint8_t cs_release:1;//片选释放
};

struct spi_cs
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
};

struct spi_priv
{
    SPI_TypeDef *s;
    struct spi_cs *cs;
    uint8_t mode;
    uint8_t data_width;
};

static struct spi_cs spi2_cs =
{
    SPI2_NSS_PORT,SPI2_NSS_PIN
};

static struct spi_priv mSpi[SPI_COM_MAX] =
{
    {SPI1,(void*)0,SPI_SPI_MODE_3 | SPI_SPI_MSB,8},
    {SPI2,&spi2_cs,SPI_SPI_MODE_3 | SPI_SPI_MSB,8}
};

static void SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    /* 使能 gpio时钟 */
    RCC_APB2PeriphClockCmd(SPI1_SCK_APB | SPI1_MISO_APB | SPI1_MOSI_APB,ENABLE);
    /* 使能 SPI时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
    GPIO_Init(SPI1_SCK_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
    GPIO_Init(SPI1_MISO_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN;
    GPIO_Init(SPI1_MOSI_PORT,&GPIO_InitStructure);

    if(mSpi[SPI_COM1].data_width <= 8)
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    else if(mSpi[SPI_COM1].data_width <= 16)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    }
    /* 相位极性 */
    if(mSpi[SPI_COM1].mode & SPI_SPI_CPOL)
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; /* 空闲状态为高电平 */
    else
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    }
    if(mSpi[SPI_COM1].mode & SPI_SPI_CPHA)
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; /* 第二个跳边沿采样 */
    else
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    if(mSpi[SPI_COM1].mode & SPI_SPI_MSB)
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; /* 高位 先发送 */
    else
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; /*全双工 */
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; /*主模式 */
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; /* 软件还是硬件 片选 */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1,&SPI_InitStructure);
    //SPI_CalculateCRC(SPI1,DISABLE);//禁止多项式
    SPI_Cmd(SPI1,ENABLE);
}

static void SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    /* 使能 gpio时钟 */
    RCC_APB2PeriphClockCmd(SPI2_SCK_APB | SPI2_MISO_APB | SPI2_MOSI_APB | SPI2_NSS_APB,ENABLE);
    /* 使能 SPI时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = SPI2_NSS_PIN;
    GPIO_Init(SPI2_NSS_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN;
    GPIO_Init(SPI2_SCK_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN;
    GPIO_Init(SPI2_MISO_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN;
    GPIO_Init(SPI2_MOSI_PORT,&GPIO_InitStructure);

    if(mSpi[SPI_COM2].data_width <= 8)
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    else if(mSpi[SPI_COM2].data_width <= 16)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    }
    /* 相位极性 */
    if(mSpi[SPI_COM2].mode & SPI_SPI_CPOL)
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; /* 空闲状态为高电平 */
    else
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    }
    if(mSpi[SPI_COM2].mode & SPI_SPI_CPHA)
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; /* 第二个跳边沿采样 */
    else
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    if(mSpi[SPI_COM2].mode & SPI_SPI_MSB)
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; /* 高位 先发送 */
    else
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; /*全双工 */
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; /*主模式 */
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; /* 软件还是硬件 片选 */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI2,&SPI_InitStructure);
    //SPI_CalculateCRC(SPI1,DISABLE);//禁止多项式
    SPI_Cmd(SPI2,ENABLE);
}

void bsp_spi_init()
{
    SPI_I2S_DeInit(SPI1);
    SPI1_Init();

    SPI_I2S_DeInit(SPI2);
    SPI2_Init();
}

static uint16_t xfer(SpiCom iNum,struct spi_message *msg)
{
    uint16_t size = msg->length;
    uint16_t dat;
    uint8_t *send_ptr = msg->send_buf;
    uint8_t *recv_ptr = msg->recv_buf;

    if(msg->cs_take && mSpi[iNum].cs)
    {
        mSpi[iNum].cs->GPIOx->BRR = mSpi[iNum].cs->GPIO_Pin; /* 拉低 */
    }

    if(mSpi[iNum].data_width == 8)
    {
        while(size--)
        {
            if(send_ptr != (void*)0)
            {
                dat = *send_ptr++;
            }
            while(SPI_I2S_GetFlagStatus(mSpi[iNum].s,SPI_I2S_FLAG_TXE) == RESET) {}
            SPI_I2S_SendData(mSpi[iNum].s,(uint8_t)dat);
            while(SPI_I2S_GetFlagStatus(mSpi[iNum].s,SPI_I2S_FLAG_RXNE) == RESET) {}

            dat = SPI_I2S_ReceiveData(mSpi[iNum].s);

            if(recv_ptr != (void*)0)
            {
                *recv_ptr++ = (uint8_t)dat;
            }
        }
    }
    else if(mSpi[iNum].data_width == 16)
    {
        while(size--)
        {
            if(send_ptr != (void*)0)
            {
                dat = *send_ptr++;
            }
            while(SPI_I2S_GetFlagStatus(mSpi[iNum].s,SPI_I2S_FLAG_TXE) == RESET) {}
            SPI_I2S_SendData(mSpi[iNum].s,dat);
            while(SPI_I2S_GetFlagStatus(mSpi[iNum].s,SPI_I2S_FLAG_RXNE) == RESET) {}

            dat = SPI_I2S_ReceiveData(mSpi[iNum].s);

            if(recv_ptr != (void*)0)
            {
                *recv_ptr++ = dat;
            }
        }
    }

    /* release */
    if(msg->cs_release && mSpi[iNum].cs)
    {
        mSpi[iNum].cs->GPIOx->BSRR = mSpi[iNum].cs->GPIO_Pin;
    }

    return msg->length;
}

uint16_t spi_transfer(SpiCom iNum,void *iFrameSend,void *iFrameRecv,uint32_t len)
{
    struct spi_message message;
    message.send_buf = iFrameSend;
    message.recv_buf = iFrameRecv;
    message.length = len;
    message.cs_release = 1;
    message.cs_take = 1;

    return xfer(iNum,&message);
}
