#ifndef __BSP_STRING_H
#define __BSP_STRING_H

#include <stdint.h>

/******************************************************************************
* 函数名 : HexToAscii()
* 描  述 : 得到单字节ASCII
* 输  入 : data,待转换数，低4位有效
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/
uint8_t HexToAscii(uint8_t data);

/******************************************************************************
* 函数名 : HexToStr()
* 描  述 : 得到单字节ASCII
* 输  入 : data,待转换数，低4位有效
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/
uint8_t HexToStr(uint8_t *ptr,uint32_t dat,uint8_t len,uint8_t zs);

/******************************************************************************
* 函数名 : HexToBcdStr()
* 描  述 : 得到单字节ASCII
* 输  入 : data,待转换数，低4位有效
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/
uint8_t HexToBcdStr(uint8_t *ptr,int32_t dat,uint8_t len,uint8_t zs);
uint8_t uHexToBcdStr(uint8_t *ptr,uint32_t dat,uint8_t len,uint8_t zs);

/******************************************************************************
* 函数名 : StrToHex()
* 描  述 : 得到HEX码
* 输  入 : data,待转换数
* 输  出 : 无
* 返回值 : 单字节hex码
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint32_t StrToHex(uint8_t *ptr,uint8_t len);

/******************************************************************************
* 函数名 : HexToBcd()
* 描  述 : 得到HEX码
* 输  入 : data,待转换数
* 输  出 : 无
* 返回值 : 单字节hex码
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint32_t HexToBcd(uint32_t Num);

void GetYearMonDayStr(uint8_t *str,struct tm *Time);

/******************************************************************************
* 函数名 : BitToChar()
* 描  述 : 位数据转换成数据
* 输  入 : dat,待转换数
* 输  出 : 无
* 返回值 : 位转换成数据
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint8_t BitToChar(uint8_t dat);
/******************************************************************************
* 函数名 : CharToBit()
* 描  述 : 数据转换成位数据
* 输  入 : dat,待转换数
* 输  出 : 转换成位数据
* 返回值 : 转换成位数据
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint8_t CharToBit(uint8_t dat);

int atoi_m(const char *s);
long atol_m(const char *s);

/****************** string ***********************/

void *memset_m(void *s,int c,unsigned long n);

void *memcpy_m(void *dest,const void *src,unsigned long n);

void *memmove_m(void *dest,const void *src,unsigned long n);

void *memcmp_m(const void *s1,const void *s2,unsigned long n);

int tolower_m(int c);

int toupper_m(int c);

int strcmp_m(const char *s1,const char *s2);

int strncmp_m(const char *cs,const char *ct,unsigned long count);

int strcasecmp_m(const char *a,const char *b);

int strncasecmp_m(const char *cs,const char *ct,unsigned long count);

int sscanf_m(const char *buf,const char *fmt,...);

unsigned long strlen_m(const char *s);

char *strstr_m(const char *s1,const char *s2);

char *strcpy_m(char *dest,const char *src);

unsigned long strlen_m(const char *s);

unsigned long strnlen_m(const char *s,unsigned long maxlen);

char *strncpy_m(char *dest,const char *src,unsigned long n);

unsigned long strlcpy_m(char *dst,const char *src,unsigned long count);

char *strncat_m(char *dest,const char *src,unsigned long count);

char *strcat_m(char *dest,const char *src);

char *strchr_m(const char *s1,int i);

char *strrchr_m(const char *t,int c);

char *strtok_m(char *s,const char *delim);

char *strtok_r_m(char *s,const char *delim,char **ptrptr);

unsigned long strcspn_m(const char *s,const char *reject);

unsigned long strspn_m(const char *s,const char *accept);

long strtol(const char *str,char **endptr,int base);

long long strtoll(const char *str,char **endptr,int base);

int atoi(const char *s);
int atol(const char *s);


#endif // MINI_LIBC_H
