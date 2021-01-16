#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdint.h>

/*!< stm32 内部flash读写操作 */

#define FLASH_SIZE      128         /*!< flash容量 (单位K) */
#define FLASH_WRITE_EN      1       /*!< flash写入使能 */

#define FLASH_BASE      0x0800000   /*!< flash起始地址 */

uint16_t flash_read_halfword(uint32_t faddr); /*!< 读出半字 */

void flash_write(uint32_t waddr,uint16_t *pBuffer,uint16_t num);

void flash_read(uint32_t raddr,uint16_t *pBuffer,uint16_t num);

void flash_write_analy(uint32_t waddr,uint16_t *pBuffer,uint16 num);

#endif
