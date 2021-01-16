
#include "bsp_i2c.h"

#define I2C_ACK_TIMEOUT         10 /*!< I2C�ȴ�Ӧ��ʱʱ�� */
#define SDA_OUT_OD_CON_WORD     0x0101 /*!< ��©��� */
#define SDA_IN_FLOAT_CON_WORD   0x0100 /*!< �������� */

static void delay_us(uint16_t us)
{

}

static void i2c_SCL_Set(I2CPort_Struct *p)
{
    p->scl_port->BSRR = p->scl_pin; /* SCL�ߵ�ƽ */
}

static void i2c_SCL_Clear(I2CPort_Struct *p)
{
    p->scl_port->BRR = p->scl_pin;
}

static void i2c_SDA_Set(I2CPort_Struct *p)
{
    p->sda_port->BSRR = p->sda_pin; /* SDA�ߵ�ƽ */
}

static void i2c_SDA_Clear(I2CPort_Struct *p)
{
    p->sda_port->BRR = p->sda_pin;
}

/**
 * @brief ��ȡSDA�ϵĸߵ�ƽ״̬
 */
static uint8_t i2c_SDA_Read(I2CPort_Struct *p)
{
    return GPIO_ReadInputDataBit(p->sda_port,p->sda_pin);
}

/**
  * @brief  SDA�����߷����л�
  * @param  p:IIC��������
  * @param  dir:��������
  * ֻȡ���������궨��
  * @arg SDA_OUT_OD_CON_WORD ��©���
  * @arg SDA_IN_FLOAT_CON_WORD ��������
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
  * @brief  IIC���������ź�
  * @param  p:IIC��������
  * @retval NONE
*/
void i2c_Start(I2CPort_Struct *p)
{/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SDA_Set(p);   /*����������*/
    i2c_SCL_Set(p);   /*ʱ��������*/
    delay_us(5);    /*Ҫ�����4.7us*/
    i2c_SDA_Clear(p); /*����������*/
    delay_us(2);
    i2c_SCL_Clear(p); /*ʱ��������*/
}
/**
  * @brief  IIC����ֹͣ�ź�
  * @param  p:IIC��������
  * @retval NONE
*/
void i2c_Stop(I2CPort_Struct *p)
{/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SDA_Clear(p); /*����������*/
    i2c_SCL_Set(p);   /*ʱ��������*/
    delay_us(5);    /*Ҫ�����4.7us*/
    i2c_SDA_Set(p);   /*����������*/
    delay_us(2);
}
/**
  * @brief  ��λIIC����
  * @param  p:IIC��������
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
        i2c_SCL_Clear(p); /*ʱ��������*/
        delay_us(delay);
        i2c_SCL_Set(p);   /*ʱ��������*/
        delay_us(delay);
    }
    i2c_SCL_Clear(p); /*ʱ��������*/
    i2c_Start(p);
    i2c_Stop(p);
}
/**
  * @brief  IIC����Ӧ���ź�
  * @param  p:IIC��������
  * @retval NONE
*/
static void i2c_Ack(I2CPort_Struct *p)
{
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SCL_Clear(p); /*ʱ��������*/
    i2c_SDA_Clear(p); /*����������*/
    delay_us(delay);
    i2c_SCL_Set(p);   /*ʱ��������*/
    delay_us(delay);
    i2c_SCL_Clear(p); /*ʱ��������*/
}

/**
  * @brief  IIC���߷�Ӧ���ź�
  * @param  p:IIC��������
  * @retval NONE
*/
static void i2c_NAck(I2CPort_Struct *p)
{
    i2c_sda_dir_change(p,SDA_OUT_OD_CON_WORD);
    i2c_SCL_Clear(p); /*ʱ��������*/
    i2c_SDA_Set(p);   /*����������*/
    delay_us(1);
    i2c_SCL_Set(p);   /*ʱ��������*/
    delay_us(1);
    i2c_SCL_Clear(p); /*ʱ��������*/
}
/**
  * @brief  IIC���ߵȴ�Ӧ��
  * @param  p:IIC��������
  * @retval I2C_OK �յ�Ӧ��
  * @retval I2C_ERR ��Ӧ��
*/
uint8_t i2c_WaitAck(I2CPort_Struct *p)
{/*�ȴ�Ӧ��*/
    uint8_t res;
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    uint16_t temp=0;
    i2c_SDA_Set(p);	/* CPU�ͷ�SDA���� */
    i2c_sda_dir_change(p,SDA_IN_FLOAT_CON_WORD);/*����Ϊ����*/
    i2c_SCL_Set(p);	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
    temp = I2C_ACK_TIMEOUT;
    while(temp>0)
    {
        if (i2c_SDA_Read(p))	/* CPU��ȡSDA����״̬ */
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
    i2c_SDA_Clear(p); /*����������*/
    return res;
}
/**
  * @brief  ��IIC���߷���һ���ֽ�����
  * @param  p:IIC��������
  * @param  dat:Ҫ���͵�����
  * @retval NONE
*/
void i2c_SendOneByte(I2CPort_Struct *p,uint8_t dat)
{
    uint8_t i;
    uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    /* �ȷ����ֽڵĸ�λbit7 */
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
        dat <<= 1;	/* ����һ��bit */
    }
}
/**
  * @brief  ��IIC���߶�ȡһ���ֽ�����
  * @param  p:IIC��������
  * @retval ����������
*/
uint8_t i2c_ReadOneByte(I2CPort_Struct *p)
{
    uint8_t i;
    uint8_t value;
  uint8_t delay = (p->freq%2)?(p->freq/2+1):(p->freq/2);
    /* ������1��bitΪ���ݵ�bit7 */
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
  * @brief  ��IIC�ӻ��豸д��N�ֽ�
  * @param  p:IIC���߽ӿ�����
  * @param  buf:��Ҫд�����ݵĻ����ַ
  * @param  buflen:��Ҫд����ֽ���
  * @param  startstop:�����Ƿ����������ֹͣ�ź�
  *       startstop����Ϊ���¼�����
  * @arg IIC_NOSTART_NOSTOP   д����֮ǰ�����������ߣ�д���ֹͣ����
  * @arg IIC_NOSTART_STOP     д����֮ǰ�����������ߣ�д���ֹͣ����
  * @arg IIC_START_NOSTOP     д����֮ǰ���������ߣ�д���ֹͣ����
  * @arg IIC_START_STOP       д����֮ǰ���������ߣ�д���ֹͣ����
  * @retval TRUE�������ɹ�
  * @retval FALSE������ʧ��
*/
uint8_t i2c_WriteNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen,uint8_t startstop)
{
    uint16_t i=0;
  uint8_t res=1;
  uint8_t ack=I2C_OK;
  if((startstop==IIC_START_NOSTOP)||(startstop==IIC_START_STOP))
  {
    i2c_Start(p);/*��������*/
    i2c_SendOneByte(p,(p->device_addr<<1)|IIC_OPT_WRITE);/*���͵�ַ��Ŀ���豸д��N���ֽ�*/
    ack = i2c_WaitAck(p);
  }
  if(ack==I2C_OK)/*�ȴ�Ӧ��*/
  {
    i=0;
    while(i<buflen)
    {
      i2c_SendOneByte(p,buf[i++]);/*�������ݵ�����*/
      if(i2c_WaitAck(p)!=I2C_OK)
      {
        res = 0;
      }
    }
  }
  else res = 0;
  if((startstop==IIC_NOSTART_STOP)||(startstop==IIC_START_STOP))
  {
    i2c_Stop(p);/*ֹͣ*/
  }
  return res;/*�����Ѿ����͵��ֽ���*/
}

/**
  * @brief  ��IIC���߶�ȡN���ֽ�
  * @param  p:IIC���߽ӿ�����
  * @param  buf:���ݵĻ����ַ
  * @param  buflen:��Ҫ��ȡ���ֽ���
  * @retval ʵ�ʶ��������ݸ���
*/
uint16_t i2c_ReadNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen)
{
    uint16_t i=0;
  i2c_Start(p);/*��������*/
  i2c_SendOneByte(p,(p->device_addr<<1)|IIC_OPT_READ);/*���͵�ַ��Ŀ���豸����N���ֽ�*/
  if(i2c_WaitAck(p)==I2C_OK)/*�ȴ�Ӧ��*/
  {
    i=0;
    while(i<buflen)
    {
      buf[i++] = i2c_ReadOneByte(p);/*�����߶�һ���ֽ�*/

      if(i<buflen)i2c_Ack(p);/*����Ӧ���ź�*/
      else i2c_NAck(p);/*������Ӧ���ź�*/
    }
  }
  i2c_Stop(p);/*ֹͣ*/
  return i;/*�����Ѿ��������ֽ���*/
}
