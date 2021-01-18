#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include <stdint.h>

#define SPI_SPI_CPHA     (1<<0)                             /* bit[0]:CPHA, clock phase 相位*/
#define SPI_SPI_CPOL     (1<<1)                             /* bit[1]:CPOL, clock polarity 极性*/

#define SPI_SPI_LSB      (0<<2)                             /* bit[2]: 0-LSB 低位有效*/
#define SPI_SPI_MSB      (1<<2)                             /* bit[2]: 1-MSB */

#define SPI_SPI_MASTER   (0<<3)                             /* SPI master device */
#define SPI_SPI_SLAVE    (1<<3)                             /* SPI slave device */

#define SPI_SPI_MODE_0       (0 | 0)                        /* CPOL = 0, CPHA = 0 */
#define SPI_SPI_MODE_1       (0 | SPI_SPI_CPHA)                 /* CPOL = 0, CPHA = 1 */
#define SPI_SPI_MODE_2       (SPI_SPI_CPOL | 0)                 /* CPOL = 1, CPHA = 0 */
#define SPI_SPI_MODE_3       (SPI_SPI_CPOL | SPI_SPI_CPHA)          /* CPOL = 1, CPHA = 1 */

#define SPI_MODE_MASK    (SPI_SPI_CPHA | SPI_SPI_CPOL | SPI_SPI_MSB)

#define SPI_SPI_CS_HIGH  (1<<4)                             /* Chipselect active high */
#define SPI_SPI_NO_CS    (1<<5)                             /* No chipselect */
#define SPI_SPI_3WIRE    (1<<6)                             /* SI/SO pin shared */
#define SPI_SPI_READY    (1<<7)                             /* Slave pulls low to pause */

typedef enum
{
    SPI_COM1,
    SPI_COM2,
    SPI_COM_MAX
}SpiCom;


void bsp_spi_init(void);

uint16_t spi_transfer(SpiCom iNum,void *iFrameSend,void *iFrameRecv,uint32_t len);

#endif
