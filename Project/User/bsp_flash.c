#include "stm32f10x_flash.h"

#include "bsp_flash.h"

/**
 * @brief 读取指定地址的半字数据(16bit)
 * @param faddr 该地址必须为2的倍数
 * @return
 */
uint16_t flash_read_halfword(uint32_t faddr)
{
    return *(uint16_t*)faddr;
}

void flash_read(uint32_t raddr,uint16_t *pBuffer,uint16_t num)
{
    uint16_t i;
    for(i = 0;i < num;i++)
    {
        pBuffer[i] = flash_read_halfword(raddr); /* 读取两个字节 */
        raddr += 2;
    }
}

void flash_write_analy(uint32_t waddr,uint16_t *pBuffer,uint16_t num)
{
    uint8_t i;
    if((waddr / 1024) == ((waddr+num*2) / 1024))
    {
        flash_write(waddr,pBuffer,num);
    }
    else
    {
        for(i = 0;i < (num * 2);i++)
        {
            if((waddr+i) % 1024 == 0)
                break;
        }
        flash_write(waddr,pBuffer,i/2);
        flash_write(waddr+i,(pBuffer+i/2),num-(i/2));
    }
}

#if FLASH_WRITE_EN
/**
 * @brief flash_write_nocheck  写入不检查
 * @param waddr 起始地址
 * @param pBuffer 数据
 * @param num 半字个数
 */
void flash_write_nocheck(uint32_t waddr,uint16_t *pBuffer,uint16_t num)
{
    uint16_t i;
    for(i = 0;i < num;i++)
    {
        FLASH_ProgramHalfWord(waddr,pBuffer[i]);
        waddr += 2; /* 地址增加 2 */
    }
}
#endif

#if FLASH_SIZE < 256
#define SECTOR_SIZE 1024 /* 字节 */
#else
#define SECTOR_SIZE 2048
#endif

uint16_t flashBuffer[SECTOR_SIZE / 2]; /* 最多2K字节 */

void flash_write(uint32_t waddr,uint16_t *pBuffer,uint16_t num)
{
    uint32_t secpos; /* 扇区地址 */
    uint16_t secoff; /* 扇区内偏移地址 16bit计算 */
    uint16_t secremain; /* 扇区内剩余地址 16bit 计算 */
    uint16_t i;
    uint32_t offaddr; /* 去掉 0x08000000后的地址 */

    if(waddr < FLASH_SIZE || (waddr >= (MACRO_FLASH_BASE + 1024*FLASH_SIZE)))
        return ; /* 地址非法 */

    FLASH_Unlock(); /* 解锁 */
    offaddr = waddr - MACRO_FLASH_BASE; /* 实际偏移地址 */
    secpos = offaddr / SECTOR_SIZE; /* 扇区地址 0~127 */
    secoff = (offaddr % SECTOR_SIZE) / 2; /* 在扇区内的偏移(2个字节为基本单位)*/
    secremain = SECTOR_SIZE / 2 - secoff; /* 扇区剩余空间大小 */

    if(num <= secremain)
        secremain = num; /* 不大于该扇区范围 */
    while(1)
    {
        /* 读取整个扇区的内容 */
        flash_read(secpos*SECTOR_SIZE+MACRO_FLASH_BASE,flashBuffer,SECTOR_SIZE/2);
        for(i=0;i < secremain;i++) /* 校验数据 */
        {
            if(flashBuffer[secoff+i] != 0xFFFF)
                break; /*需要擦除 */
        }
        if(i < secremain) /*需要擦除 */
        {
            FLASH_ErasePage(secpos * SECTOR_SIZE + MACRO_FLASH_BASE); /* 擦除这个删除 */
            for(i = 0;i < secremain;i++)
            {
                /* 复制 */
                flashBuffer[i + secoff] = pBuffer[i];
            }
            /* 写入整个扇区 */
            flash_write_nocheck(secpos * SECTOR_SIZE + MACRO_FLASH_BASE,flashBuffer,SECTOR_SIZE / 2);
        }
        else
        {
            /* 写已经擦除了的,直接写入扇区剩余区间 */
            flash_write_nocheck(waddr,pBuffer,secremain);
        }
        if(num == secremain)
        {
            break; /* 写入结束了 */
        }
        else
        {
            /* 写入未结束 */
            secpos++; /* 扇区地址增加1 */
            secoff = 0; /* 扇区偏移=0 */
            pBuffer += secremain; /*指针偏移 */
            waddr += secremain; /* 写地址偏移 */
            num -= secremain; /* 字节数递减 */
            if(num > (SECTOR_SIZE / 2))
            {
                secremain = SECTOR_SIZE / 2; /*下一个扇区还是写不完 */
            }
            else
            {
                secremain = num; /*下一个扇区可以写完了 */
            }
        }
    }

    FLASH_Lock(); /* 上锁 */
}
