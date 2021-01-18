#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f10x_gpio.h"

#define IIC_OPT_WRITE       0x0 /*!< iic写操作 */
#define IIC_OPT_READ        0x01 /*!< iic读操作 */

#define I2C_OK          1
#define I2C_ERR         0

#define IIC_NOSTART_NOSTOP    1/*!< 写数据之前，不启动总线，写完后不停止总线*/
#define IIC_NOSTART_STOP      2/*!< 写数据之前，不启动总线，写完后停止总线*/
#define IIC_START_NOSTOP      3/*!< 写数据之前，启动总线，写完后不停止总线*/
#define IIC_START_STOP        4/*!< 写数据之前，启动总线，写完后停止总线*/

typedef struct
{
    GPIO_TypeDef* scl_port; /*!< SCL脚的端口*/
    GPIO_TypeDef* sda_port; /*!< SDA脚的端口*/
    uint16_t scl_pin        /*!< SCL引脚编号*/;
    uint16_t sda_pin        /*!< SDA引脚编号*/;
    uint8_t device_addr;    /*!< 设备地址*/
    uint8_t freq;           /*!< 总线速度，单位为1us，如100K的速度，该参数为10us*/
}I2CPort_Struct;/*!< IIC总线接口数据结构*/

void      i2c_Stop(I2CPort_Struct *p);
uint16_t  i2c_ReadNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen);
uint8_t   i2c_WriteNByte(I2CPort_Struct *p,uint8_t *buf,uint16_t buflen,uint8_t startstop);

#endif
