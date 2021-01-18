#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f10x_gpio.h"

#define IIC_OPT_WRITE       0x0 /*!< iicд���� */
#define IIC_OPT_READ        0x01 /*!< iic������ */

#define I2C_OK          1
#define I2C_ERR         0

#define IIC_NOSTART_NOSTOP    1/*!< д����֮ǰ�����������ߣ�д���ֹͣ����*/
#define IIC_NOSTART_STOP      2/*!< д����֮ǰ�����������ߣ�д���ֹͣ����*/
#define IIC_START_NOSTOP      3/*!< д����֮ǰ���������ߣ�д���ֹͣ����*/
#define IIC_START_STOP        4/*!< д����֮ǰ���������ߣ�д���ֹͣ����*/

typedef struct
{
    GPIO_TypeDef* scl_port; /*!< SCL�ŵĶ˿�*/
    GPIO_TypeDef* sda_port; /*!< SDA�ŵĶ˿�*/
    uint16_t scl_pin        /*!< SCL���ű��*/;
    uint16_t sda_pin        /*!< SDA���ű��*/;
    uint8_t device_addr;    /*!< �豸��ַ*/
    uint8_t freq;           /*!< �����ٶȣ���λΪ1us����100K���ٶȣ��ò���Ϊ10us*/
}I2CPort_Struct;/*!< IIC���߽ӿ����ݽṹ*/

void      i2c_Stop(I2CPort_Struct *p);
uint16_t  i2c_ReadNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen);
uint8_t   i2c_WriteNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen,uint8_t startstop);

#endif
