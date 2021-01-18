#include "stm32f10x_flash.h"

#include "bsp_flash.h"

/**
 * @brief ��ȡָ����ַ�İ�������(16bit)
 * @param faddr �õ�ַ����Ϊ2�ı���
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
        pBuffer[i] = flash_read_halfword(raddr); /* ��ȡ�����ֽ� */
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
 * @brief flash_write_nocheck  д�벻���
 * @param waddr ��ʼ��ַ
 * @param pBuffer ����
 * @param num ���ָ���
 */
void flash_write_nocheck(uint32_t waddr,uint16_t *pBuffer,uint16_t num)
{
    uint16_t i;
    for(i = 0;i < num;i++)
    {
        FLASH_ProgramHalfWord(waddr,pBuffer[i]);
        waddr += 2; /* ��ַ���� 2 */
    }
}
#endif

#if FLASH_SIZE < 256
#define SECTOR_SIZE 1024 /* �ֽ� */
#else
#define SECTOR_SIZE 2048
#endif

uint16_t flashBuffer[SECTOR_SIZE / 2]; /* ���2K�ֽ� */

void flash_write(uint32_t waddr,uint16_t *pBuffer,uint16_t num)
{
    uint32_t secpos; /* ������ַ */
    uint16_t secoff; /* ������ƫ�Ƶ�ַ 16bit���� */
    uint16_t secremain; /* ������ʣ���ַ 16bit ���� */
    uint16_t i;
    uint32_t offaddr; /* ȥ�� 0x08000000��ĵ�ַ */

    if(waddr < FLASH_SIZE || (waddr >= (MACRO_FLASH_BASE + 1024*FLASH_SIZE)))
        return ; /* ��ַ�Ƿ� */

    FLASH_Unlock(); /* ���� */
    offaddr = waddr - MACRO_FLASH_BASE; /* ʵ��ƫ�Ƶ�ַ */
    secpos = offaddr / SECTOR_SIZE; /* ������ַ 0~127 */
    secoff = (offaddr % SECTOR_SIZE) / 2; /* �������ڵ�ƫ��(2���ֽ�Ϊ������λ)*/
    secremain = SECTOR_SIZE / 2 - secoff; /* ����ʣ��ռ��С */

    if(num <= secremain)
        secremain = num; /* �����ڸ�������Χ */
    while(1)
    {
        /* ��ȡ�������������� */
        flash_read(secpos*SECTOR_SIZE+MACRO_FLASH_BASE,flashBuffer,SECTOR_SIZE/2);
        for(i=0;i < secremain;i++) /* У������ */
        {
            if(flashBuffer[secoff+i] != 0xFFFF)
                break; /*��Ҫ���� */
        }
        if(i < secremain) /*��Ҫ���� */
        {
            FLASH_ErasePage(secpos * SECTOR_SIZE + MACRO_FLASH_BASE); /* �������ɾ�� */
            for(i = 0;i < secremain;i++)
            {
                /* ���� */
                flashBuffer[i + secoff] = pBuffer[i];
            }
            /* д���������� */
            flash_write_nocheck(secpos * SECTOR_SIZE + MACRO_FLASH_BASE,flashBuffer,SECTOR_SIZE / 2);
        }
        else
        {
            /* д�Ѿ������˵�,ֱ��д������ʣ������ */
            flash_write_nocheck(waddr,pBuffer,secremain);
        }
        if(num == secremain)
        {
            break; /* д������� */
        }
        else
        {
            /* д��δ���� */
            secpos++; /* ������ַ����1 */
            secoff = 0; /* ����ƫ��=0 */
            pBuffer += secremain; /*ָ��ƫ�� */
            waddr += secremain; /* д��ַƫ�� */
            num -= secremain; /* �ֽ����ݼ� */
            if(num > (SECTOR_SIZE / 2))
            {
                secremain = SECTOR_SIZE / 2; /*��һ����������д���� */
            }
            else
            {
                secremain = num; /*��һ����������д���� */
            }
        }
    }

    FLASH_Lock(); /* ���� */
}
