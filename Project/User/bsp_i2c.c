
#include "bsp_i2c.h"

#define I2C_ACK_TIMEOUT         10 /*!< I2C等待应答超时时间 */
#define SDA_OUT_OD_CON_WORD     0x0101 /*!< 开漏输出 */
#define SDA_IN_FLOAT_CON_WORD   0x0100 /*!< 浮空输入 */

static void delay_us(uint16_t us)
{

}

static void i2c_SCL_Set(I2CPort_Struct *p)
{
    p->scl_port->BSRR = p->scl_pin; /* SCL高电平 */
}

static void i2c_SCL_Clear(I2CPort_Struct *p)
{
    p->scl_port->BRR = p->scl_pin;
}

static void i2c_SDA_Set(I2CPort_Struct *p)
{
    p->sda_port->BSRR = p->sda_pin; /* SDA高电平 */
}

static void i2c_SDA_Clear(I2CPort_Struct *p)
{
    p->sda_port->BRR = p->sda_pin;
}

/**
 * @brief 读取SDA上的高电平状态
 */
static uint8_t i2c_SDA_Read(I2CPort_Struct *p)
{
    return GPIO_ReadInputDataBit(p->sda_port,p->sda_pin);
}

/**
  * @brief  SDA数据线方向切换
  * @param  p:IIC总线数据
  * @param  dir:方向数据
  * 只取以下两个宏定义
  * @arg SDA_OUT_OD_CON_WORD 开漏输出
  * @arg SDA_IN_FLOAT_CON_WORD 浮空输入
  * @retval NONE
*/
static void i2c_sda_dir_change(I2CPort_Struct *p,uint32_t dir)
{
    uint32_t temp = 0;

    temp = (p->sda_pin&0xff)?(p->sda_pin):(((p->sda_pin)>>8)&0xff);
    switch(temp)
    {
        case 0x01:temp = 0;break;
        case 0x02:temp = 1;break;
        case 0x04:temp = 2;break;
        case 0x08:temp = 3;break;
        case 0x10:temp = 4;break;
        case 0x20:temp = 5;break;
        case 0x40:temp = 6;break;
        case 0x80:temp = 7;break;
    default:temp=0xff;break;
    }
  if(temp==0xff)return;
    if((p->sda_pin&0xff) != 0)
    {
        p->sda_port->CRL |= dir<<(temp*4);
        p->sda_port->CRL &= ~(dir<<(temp*4));
    }
    else if((p->sda_pin&0x00ff) != 0)
    {
        p->sda_port->CRH |= dir<<(temp*4);
        p->sda_port->CRH &= ~(dir<<(temp*4)); /* ?? */
    }
}

/**
  * @brief  IIC总线启动信号
  * @param  p:IIC总线数据
  * @retval NONE
*/
void i2c_Start(I2CPort_Struct *p)
{/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SDA_Set(p);   /*数据线拉高*/
    i2c_SCL_Set(p);   /*时钟线拉高*/
    delay_us(5);    /*要求大于4.7us*/
    i2c_SDA_Clear(p); /*数据线拉低*/
    delay_us(2);
    i2c_SCL_Clear(p); /*时钟线拉低*/
}
/**
  * @brief  IIC总线停止信号
  * @param  p:IIC总线数据
  * @retval NONE
*/
void i2c_Stop(I2CPort_Struct *p)
{/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SDA_Clear(p); /*数据线拉低*/
    i2c_SCL_Set(p);   /*时钟线拉高*/
    delay_us(5);    /*要求大于4.7us*/
    i2c_SDA_Set(p);   /*数据线拉高*/
    delay_us(2);
}
/**
  * @brief  复位IIC总线
  * @param  p:IIC总线数据
  * @retval NONE
*/
void i2c_Reset(I2CPort_Struct *p)
{
    uint8_t i=0;
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    i2c_Start(p);
    i2c_SDA_Set(p);
    for(i=0;i<9;i++)
    {
        i2c_SCL_Clear(p); /*时钟线拉低*/
        delay_us(delay);
        i2c_SCL_Set(p);   /*时钟线拉高*/
        delay_us(delay);
    }
    i2c_SCL_Clear(p); /*时钟线拉低*/
    i2c_Start(p);
    i2c_Stop(p);
}
/**
  * @brief  IIC总线应答信号
  * @param  p:IIC总线数据
  * @retval NONE
*/
static void i2c_Ack(I2CPort_Struct *p)
{
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SCL_Clear(p); /*时钟线拉低*/
    i2c_SDA_Clear(p); /*数据线拉低*/
    delay_us(delay);
    i2c_SCL_Set(p);   /*时钟线拉高*/
    delay_us(delay);
    i2c_SCL_Clear(p); /*时钟线拉低*/
}

/**
  * @brief  IIC总线非应答信号
  * @param  p:IIC总线数据
  * @retval NONE
*/
static void i2c_NAck(I2CPort_Struct *p)
{
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SCL_Clear(p); /*时钟线拉低*/
    i2c_SDA_Set(p);   /*数据线拉高*/
    delay_us(1);
    i2c_SCL_Set(p);   /*时钟线拉高*/
    delay_us(1);
    i2c_SCL_Clear(p); /*时钟线拉低*/
}
/**
  * @brief  IIC总线等待应答
  * @param  p:IIC总线数据
  * @retval I2C_OK 收到应答
  * @retval I2C_ERR 无应答
*/
uint8_t i2c_WaitAck(I2CPort_Struct *p)
{/*等待应答*/
    uint8_t res;
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    uint16_t temp=0;
    i2c_SDA_Set(p);	/* CPU释放SDA总线 */
    i2c_sda_dir_change(p,SDA_IN_FLOAT_CON_WORD);/*配置为输入*/
    i2c_SCL_Set(p);	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
    temp = I2C_ACK_TIMEOUT;
    while(temp>0)
    {
        if (i2c_SDA_Read(p))	/* CPU读取SDA口线状态 */
        {
            res = I2C_ERR;
        }
        else
        {
            res = I2C_OK;
            break;
        }
        temp--;
        delay_us(2);
    }
    delay_us(delay);
    i2c_SCL_Clear(p);
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SDA_Clear(p); /*数据线拉低*/
    return res;
}
/**
  * @brief  向IIC总线发送一个字节数据
  * @param  p:IIC总线数据
  * @param  dat:要发送的数据
  * @retval NONE
*/
void i2c_SendOneByte(I2CPort_Struct *p,uint8_t dat)
{
    uint8_t i;
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    /* 先发送字节的高位bit7 */
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
        {
            i2c_SDA_Set(p);
        }
        else
        {
            i2c_SDA_Clear(p);
        }
        delay_us(delay);
        i2c_SCL_Set(p);
        delay_us(delay);
        i2c_SCL_Clear(p);
        //    delay_us(1);
        dat <<= 1;	/* 左移一个bit */
    }
}
/**
  * @brief  从IIC总线读取一个字节数据
  * @param  p:IIC总线数据
  * @retval 读到的数据
*/
uint8_t i2c_ReadOneByte(I2CPort_Struct *p)
{
    uint8_t i;
    uint8_t value;
  uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    /* 读到第1个bit为数据的bit7 */
    value = 0;
  i2c_SCL_Clear(p);
  i2c_SDA_Set(p);
  i2c_sda_dir_change(p,SDA_IN_FLOAT_CON_WORD);
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
    delay_us(delay);
        i2c_SCL_Set(p);
        delay_us(delay);
        if (i2c_SDA_Read(p))
        {
            value++;
        }
        i2c_SCL_Clear(p);
    }
  i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    return value;
}
/**
  * @brief  向IIC从机设备写入N字节
  * @param  p:IIC总线接口数据
  * @param  buf:将要写入数据的缓存地址
  * @param  buflen:将要写入的字节数
  * @param  startstop:控制是否产生启动和停止信号
  *       startstop必须为以下几个宏
  * @arg IIC_NOSTART_NOSTOP   写数据之前，不启动总线，写完后不停止总线
  * @arg IIC_NOSTART_STOP     写数据之前，不启动总线，写完后停止总线
  * @arg IIC_START_NOSTOP     写数据之前，启动总线，写完后不停止总线
  * @arg IIC_START_STOP       写数据之前，启动总线，写完后停止总线
  * @retval TRUE：操作成功
  * @retval FALSE：操作失败
*/
uint8_t i2c_WriteNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen,uint8_t startstop)
{
    uint16_t i=0;
  uint8_t res=1;
  uint8_t ack=I2C_OK;
  if((startstop==IIC_START_NOSTOP)||(startstop==IIC_START_STOP))
  {
    i2c_Start(p);/*启动总线*/
    i2c_SendOneByte(p,(p->device_addr<<1)|IIC_OPT_WRITE);/*发送地址到目标设备写入N个字节*/
    ack = i2c_WaitAck(p);
  }
  if(ack==I2C_OK)/*等待应答*/
  {
    i=0;
    while(i<buflen)
    {
      i2c_SendOneByte(p,buf[i++]);/*发送数据到总线*/
      if(i2c_WaitAck(p)!=I2C_OK)
      {
        res = 0;
      }
    }
  }
  else res = 0;
  if((startstop==IIC_NOSTART_STOP)||(startstop==IIC_START_STOP))
  {
    i2c_Stop(p);/*停止*/
  }
  return res;/*返回已经发送的字节数*/
}

/**
  * @brief  从IIC总线读取N个字节
  * @param  p:IIC总线接口数据
  * @param  buf:数据的缓存地址
  * @param  buflen:将要读取的字节数
  * @retval 实际读到的数据个数
*/
uint16_t i2c_ReadNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen)
{
    uint16_t i=0;
  i2c_Start(p);/*启动总线*/
  i2c_SendOneByte(p,(p->device_addr<<1)|IIC_OPT_READ);/*发送地址到目标设备，读N个字节*/
  if(i2c_WaitAck(p)==I2C_OK)/*等待应答*/
  {
    i=0;
    while(i<buflen)
    {
      buf[i++] = i2c_ReadOneByte(p);/*从总线读一个字节*/

      if(i<buflen)i2c_Ack(p);/*产生应答信号*/
      else i2c_NAck(p);/*产生非应答信号*/
    }
  }
  i2c_Stop(p);/*停止*/
  return i;/*返回已经读到的字节数*/
}
