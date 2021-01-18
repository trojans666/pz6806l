#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "bsp_string.h"


/******************************************************************************
* 函数名 : HexToAscii()
* 描  述 : 得到单字节ASCII
* 输  入 : data,待转换数，低4位有效
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/
const uint8_t HEX_TO_ASCII_TAB[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',' '};

uint8_t HexToAscii(uint8_t data)
{
    return (uint8_t)HEX_TO_ASCII_TAB[data&0x0f];
}

/******************************************************************************
* 函数名 : HexToStr()
* 描  述 : 得到单字节ASCII
* 输  入 :  ptr 转换后字符串地址，dat，待转换数据，len，显示数据的长度，zs，数据前是否要显示0，
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/

uint8_t HexToStr(uint8_t *ptr,uint32_t dat,uint8_t len,uint8_t zs)
{
    uint8_t num=0;
    *(ptr+len) = '\0';
    while(len--)
    {
        if((dat==0)&&(num != 0)&&(zs==0))
        {
            memcpy((ptr+len), &HEX_TO_ASCII_TAB[16], 1);
        }
        else
        {
            memcpy((ptr+len), &HEX_TO_ASCII_TAB[dat&0x0000000f], 1);
        }
        dat>>=4;
        num++;
    }
    return 1;
}

/******************************************************************************
* 函数名 : HexToBcdStr()
* 描  述 : 得到单字节ASCII
* 输  入 : ptr 转换后字符串地址，dat，待转换数据，len，显示数据的长度，zs，数据前是否要显示0，
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/

uint8_t HexToBcdStr(uint8_t *ptr,int32_t dat,uint8_t len,uint8_t zs)
{
    uint8_t num = 0,index = 0,flg = 0;
    uint32_t udat;
    if(dat<0)
    {
        flg = 1;
        udat = 0x100000000-dat;
    }
    else
    {
        udat = (uint32_t)dat;
    }
    *(ptr+len) = '\0';
    while(len--)
    {
        index = udat%10;
        if((udat==0)&&(num != 0)&&(zs==0))
        {
            memcpy((ptr+len), &HEX_TO_ASCII_TAB[16], 1);
            if(flg)
            {
                flg = 0;
                *(ptr+len) = '-';
            }

        }
        else
        {
            memcpy((ptr+len), &HEX_TO_ASCII_TAB[index], 1);
        }
        udat /=10;
        num++;
    }
    if(flg)
    {
        *ptr = '-';
    }
    return 1;
}


/******************************************************************************
* 函数名 : uHexToBcdStr()
* 描  述 : 得到单字节ASCII
* 输  入 : ptr 转换后字符串地址，dat，待转换数据，len，显示数据的长度，zs，数据前是否要显示0，
* 输  出 : 无
* 返回值 : 单字节ascii码
* 调  用 : 无
* 其  它 : 无效数据输出0x30；
******************************************************************************/

uint8_t uHexToBcdStr(uint8_t *ptr,uint32_t dat,uint8_t len,uint8_t zs)
{
    uint8_t num = 0,index = 0;
    uint32_t udat = dat;
    *(ptr + len) = '\0';
    while(len--)
    {
        index = udat%10;
        if((udat == 0)&&(num != 0)&&(zs == 0))
        {
            memcpy((ptr + len), &HEX_TO_ASCII_TAB[16], 1);
        }
        else
        {
            memcpy((ptr + len), &HEX_TO_ASCII_TAB[index], 1);
        }
        udat /=10;
        num++;
    }
    return 1;
}

/******************************************************************************
* 函数名 : StrToHex()
* 描  述 : 得到HEX码
* 输  入 : ptr,待转换字符串，len，待转换的字符串长度
* 输  出 : 无
* 返回值 : 单字节hex码
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint32_t StrToHex(uint8_t *ptr,uint8_t len)
{
    uint8_t i = 0;
    uint32_t dat=0;
    uint8_t temp=0;
    for(i=0;i<len;i++)
    {
        dat<<=4;
        temp = *(ptr+i);
        if((temp<='9')&&(temp>='0'))
        {
            temp -= '0';
        }
        else if((temp<='F')&&(temp>='A'))
        {
            temp -= 'A';
            temp += 10;
        }
        else if((temp<='f')&&(temp>='a'))
        {
            temp -= 'a';
            temp += 10;
        }
        else
            temp = 0;
        dat += (uint32_t)temp;
    }
    return dat;
}
/******************************************************************************
* 函数名 : HexToBcd()
* 描  述 : 得到HEX码
* 输  入 : data,待转换数
* 输  出 : 无
* 返回值 : 单字节hex码
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint32_t HexToBcd(uint32_t Num)
{
    uint32_t bcdData = 0;
    if(Num > 9999999)
    {
        bcdData = 0x10000000;
        Num = 10000000;
    }
    if(Num > 999999)
    {
        bcdData |= ((Num / 1000000) << 24);
        Num = Num % 1000000;
    }
    if(Num > 99999)
    {
        bcdData |= ((Num / 100000) << 20);
        Num = Num % 100000;
    }
    if(Num > 9999)
    {
        bcdData |= ((Num / 10000) << 16);
        Num = Num % 10000;
    }
    if(Num > 999)
    {
        bcdData |= ((Num / 1000) << 12);
        Num = Num % 1000;
    }
    if(Num > 99)
    {
        bcdData |= ((Num / 100) << 8);
        Num = Num % 100;
    }
    if(Num > 9)
    {
        bcdData |= ((Num / 10) <<4);
        Num = Num % 10;
    }
    bcdData |= (Num % 10);

    return bcdData;
}

/******************************************************************************
* 函数名 : GetYearMonDayStr()
* 描  述 : 获取年月日
* 输  入 : data,待转换数
* 输  出 : 无
* 返回值 : 无
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/

void GetYearMonDayStr(uint8_t *str,time_t *time)
{
    struct tm Time;
    Time = *gmtime(time);

    HexToBcdStr(str, (Time.tm_year+1900), 4, 1);
    strcpy((char*)(str+4),".");
    HexToBcdStr(str+5, (Time.tm_mon+1), 2, 1);
    strcpy((char*)(str+7),".");
    HexToBcdStr(str+8, (Time.tm_mday+1), 2, 1);
}
/******************************************************************************
* 函数名 : GethhmmStr()
* 描  述 : 获取小时分
* 输  入 : data,待转换数
* 输  出 : 无
* 返回值 : 无
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/

void GethhmmStr(uint8_t *str,time_t *time)
{
    struct tm Time = {0};
    Time = *gmtime(time);

    HexToBcdStr(str, Time.tm_hour, 2, 1);
    strcpy((char*)(str+2),":");
    HexToBcdStr(str+3, (Time.tm_min), 2, 1);
}

/******************************************************************************
* 函数名 : BitToChar()
* 描  述 : 位数据转换成数据
* 输  入 : dat,待转换数
* 输  出 : 无
* 返回值 : 位转换成数据
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
const uint8_t BIT_TAB[]={0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
uint8_t BitToChar(uint8_t dat)
{
    uint8_t i;
    for(i=0;i<sizeof(BIT_TAB);i++)
    {
        if(dat==BIT_TAB[i])
        {
            return i+1;
        }
    }
    return 0;
}
/******************************************************************************
* 函数名 : CharToBit()
* 描  述 : 数据转换成位数据
* 输  入 : dat,待转换数
* 输  出 : 转换成位数据
* 返回值 : 转换成位数据
* 调  用 : 无
* 其  它 : 无效数据输出0；
******************************************************************************/
uint8_t CharToBit(uint8_t dat)
{
    if((8>dat))
    {
        return 1<<(dat);
    }
    return 0;
}


/** 转换为16进制数组 **/
void ToHexStr(unsigned char* source, unsigned char* dest, int sourceLen)
{
    short i;
    unsigned char highByte, lowByte;

    for (i = 0; i < sourceLen; i++)
    {
        highByte = source[i] >> 4;
        lowByte = source[i] & 0x0f ;

        highByte += 0x30;

        if (highByte > 0x39)
            dest[i * 2] = highByte + 0x07;
        else
            dest[i * 2] = highByte;

        lowByte += 0x30;
        if (lowByte > 0x39)
            dest[i * 2 + 1] = lowByte + 0x07;
        else
            dest[i * 2 + 1] = lowByte;
    }
    return ;
}

#if 0
/** 自定义strtok IAR库没有 strtok_r 是由外部保存分割后剩余值。**/
char* yny_strtok(uint8_t* strToken, const uint8_t* strDelimit)
{

    //定义局部变量
    static uint8_t* text = NULL; //保存分割后的值
    uint8_t table[32];
    const uint8_t* delimit;
    uint8_t* str;
    uint8_t *head;

    //更新静态字符串
    if (strToken)
        text = strToken;

    //对不合法输入进行特殊判断
    if (text == NULL)
        return NULL;
    if (strDelimit == NULL)
        return text;

    //改变 char 为 unsigned char 以便进行位运算
    str = text;//(unsigned char*)text;
    delimit = strDelimit;//(const unsigned char*)strDelimit;

    //初始化位表
    uint8_t i;
    for (i = 0; i < 32; i++)
        table[i] = 0;
    for (; *delimit; delimit++)
    {
        table[*delimit >> 3] |= 1 << (*delimit & 7);
    }

    // 跳过分隔符直到起始位置
    while (*str && (table[*str >> 3] & (1 << (*str & 7))))
        str++;
    head = str;//(char*)str;

    // 找到第一个分隔符
    for (; *str; str++)
    {
        if (table[*str >> 3] & (1 << (*str & 7)))
        {
            *str++ = '\0';
            break;
        }
    }
    // 更新结果
    text = str;//(char*)str;
    if (*text == '\0') text = NULL;
    return head;
}

uint8_t yny_atoi(uint8_t* pstr)
{
    uint8_t Ret_Integer = 0;
    short Integer_sign = 1;

    //判断指针是否为空
    if(pstr == NULL)
    {
        return 0;
    }

    //跳过前面的空格字符
    while(*pstr == ' ')
    {
        pstr++;
    }

    /*
    * 判断正负号
    * 如果是正号，指针指向下一个字符
    * 如果是符号，把符号标记为Integer_sign置-1，然后再把指针指向下一个字符
    */
    if(*pstr == '-')
    {
        Integer_sign = -1;
    }
    if(*pstr == '-' || *pstr == '+')
    {
        pstr++;
    }

    /*
    * 把数字字符串逐个转换成整数，并把最后转换好的整数赋给Ret_Integer
    */
    while(*pstr >= '0' && *pstr <= '9')
    {
        Ret_Integer = Ret_Integer * 10 + *pstr - '0';
        pstr++;
    }
    Ret_Integer = Integer_sign * Ret_Integer;

    return Ret_Integer;
}
#endif

int atoi_m(const char *s)
{
    long int v=0;
    int sign=1;
    while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) s++;
    switch (*s)
    {
    case '-':
        sign=-1;
    case '+':
        ++s;
    }
    while ((unsigned int) (*s - '0') < 10u)
    {
        v=v*10+*s-'0';
        ++s;
    }
    return sign==-1?-v:v;
}

long atol_m(const char *s)
{
    long int v=0;
    int sign=0;
    while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) ++s;
    switch (*s)
    {
        case '-': sign=-1;
        case '+': ++s;
    }
    while ((unsigned int) (*s - '0') < 10u)
    {
        v=v*10+*s-'0'; ++s;
    }
    return sign?-v:v;
}

/*************** string ************************/

//是否为打印字符
//static int isprint_m(int ch)
//{
//    ch &= 0x7f;
//    return (ch >= 32 && ch < 127);
//}

//是否为英文字母
//static int isalpha_m(int ch)
//{
//    return (unsigned int)((ch | 0x20) - 'a') < 26u;
//}

//是否为数字
static int isdigit_m(int ch)
{
    return (unsigned int)(ch - '0') < 10u;
}

static int isspace_m(int ch)
{
    switch(ch)
    {
    case ' ':
    case '\n':
    case '\f':
    case '\r':
    case '\t':
    case '\v':
        return 1;
    default:
        return 0;
    }
}

//static int isascii_m(int ch)
//{
//    return (((unsigned char)(ch))<=0x7f);
//}

//static int toascii_m(int ch)
//{
//    return (((unsigned char)(ch))&0x7f);
//}

static int isupper_m(int ch)
{
    if(ch >= 'A' && ch <= 'Z')
        return 1;

    return 0;
}

static int islower_m(int ch)
{
    if(ch >= 'a' && ch <= 'z')
        return 1;

    return 0;
}

//是否为十六进制
static int isxdigit_m(int ch)
{
    if(isdigit_m(ch) || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F'))
        return 1;

    return 0;
}


void *memset_m(void *s,int c,unsigned long count)
{
#if 0
    char *xs = (char *)s;

    while (count--)
        *xs++ = c;

    return s;
#endif

#define LBLOCKSIZE_M      (sizeof(int))
#define UNALIGNED_M(X)    ((unsigned int)X & (LBLOCKSIZE_M - 1))
#define TOO_SMALL_M(LEN)  ((LEN) < LBLOCKSIZE_M)

    int i;
    char *m = (char *)s;
    unsigned int buffer;
    unsigned int *aligned_addr;
    unsigned int d = c & 0xff;

    if (!TOO_SMALL_M(count) && !UNALIGNED_M(s))
    {
        /* If we get this far, we know that n is large and m is word-aligned. */
        aligned_addr = (unsigned int *)s;

        /* Store D into each char sized location in BUFFER so that
         * we can set large blocks quickly.
         */
        if (LBLOCKSIZE_M == 4)
        {
            buffer = (d << 8) | d;
            buffer |= (buffer << 16);
        }
        else
        {
            buffer = 0;
            for (i = 0; i < LBLOCKSIZE_M; i ++)
                buffer = (buffer << 8) | d;
        }

        while (count >= LBLOCKSIZE_M * 4)
        {
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            count -= 4 * LBLOCKSIZE_M;
        }

        while (count >= LBLOCKSIZE_M)
        {
            *aligned_addr++ = buffer;
            count -= LBLOCKSIZE_M;
        }

        /* Pick up the remainder with a bytewise loop. */
        m = (char *)aligned_addr;
    }

    while (count--)
    {
        *m++ = (char)d;
    }

    return s;

#undef LBLOCKSIZE_M
#undef UNALIGNED_M
#undef TOO_SMALL_M
}

void *memcpy_m(void *dst,const void *src,unsigned long count)
{
#if 0
    char *tmp = (char *)dst, *s = (char *)src;
    rt_ubase_t len;

    if (tmp <= s || tmp > (s + count))
    {
        while (count--)
            *tmp ++ = *s ++;
    }
    else
    {
        for (len = count; len > 0; len --)
            tmp[len - 1] = s[len - 1];
    }

    return dst;
#endif

#define UNALIGNED_M(X, Y)                                               \
                        (((int)X & (sizeof(int) - 1)) | \
                         ((int)Y & (sizeof(int) - 1)))
#define BIGBLOCKSIZE_M    (sizeof(int) << 2)
#define LITTLEBLOCKSIZE_M (sizeof(int))
#define TOO_SMALL_M(LEN)  ((LEN) < BIGBLOCKSIZE_M)

    char *dst_ptr = (char *)dst;
    char *src_ptr = (char *)src;
    int *aligned_dst;
    int *aligned_src;
    int len = count;

    /* If the size is small, or either SRC or DST is unaligned,
    then punt into the byte copy loop.  This should be rare. */
    if (!TOO_SMALL_M(len) && !UNALIGNED_M(src_ptr, dst_ptr))
    {
        aligned_dst = (int *)dst_ptr;
        aligned_src = (int *)src_ptr;

        /* Copy 4X long words at a time if possible. */
        while (len >= BIGBLOCKSIZE_M)
        {
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            len -= BIGBLOCKSIZE_M;
        }

        /* Copy one long word at a time if possible. */
        while (len >= LITTLEBLOCKSIZE_M)
        {
            *aligned_dst++ = *aligned_src++;
            len -= LITTLEBLOCKSIZE_M;
        }

        /* Pick up any residual with a byte copier. */
        dst_ptr = (char *)aligned_dst;
        src_ptr = (char *)aligned_src;
    }

    while (len--)
        *dst_ptr++ = *src_ptr++;

    return dst;
#undef UNALIGNED_M
#undef BIGBLOCKSIZE_M
#undef LITTLEBLOCKSIZE_M
#undef TOO_SMALL_M
}

void *memmove_m(void *dest,const void *src,unsigned long n)
{
    char *tmp = (char *)dest,*s = (char *)src;

    if(s < tmp && tmp < (s + n))
    {
        tmp += n;
        s += n;

        while(n--)
        {
            *(--tmp) = *(--s);
        }
    }
    else
    {
        while(n--)
        {
            *(tmp++) = *(s++);
        }
    }
		
		return dest;
}

int memcmp_m(const void *cs,const void *ct,unsigned long count)
{
    const unsigned char *su1,*su2;
    int res = 0;

    for(su1 = cs,su2 = ct; 0 < count; ++su1,++su2,count--)
    {
        if((res = *su1 - *su2) != 0)
            break;
    }

    return res;
}

int tolower_m(int c)
{
    if(isupper_m(c))
        c -= 'A' - 'a';
    return c;
}

int toupper_m(int c)
{
    if(islower_m(c))
        c-= 'a' - 'A';
    return c;
}

int strcmp_m(const char *s1,const char *s2)
{
    while(*s1 && *s1 == *s2)
        s1++,s2++;

    return (*s1 - *s2);
}

int strncmp_m(const char *cs,const char *ct,unsigned long count)
{
    register signed char __res = 0;

    while(count)
    {
        if((__res == *cs - *ct++) != 0 || !*cs++)
            break;
        count--;
    }

    return __res;
}

int strcasecmp_m(const char *a,const char *b)
{
    int ca, cb;

    do
    {
        ca = *a++ & 0xff;
        cb = *b++ & 0xff;
        if (ca >= 'A' && ca <= 'Z')
            ca += 'a' - 'A';
        if (cb >= 'A' && cb <= 'Z')
            cb += 'a' - 'A';
    }
    while (ca == cb && ca != '\0');

    return ca - cb;
}

int strncasecmp_m(const char *s1,const char *s2,unsigned long len)
{
    register unsigned int x2;
    register unsigned int x1;
    register const char *end = s1 + len;

    while(1)
    {
        if((s1 >= end))
            return 0;

        x2 = *s2 - 'A';
        if((x2 < 26u))
            x2 += 32;

        x1 = *s1 - 'A';
        if((x1 < 26u))
            x1 += 32;

        if(x2 != x1)
            break;

        if(x1 == (unsigned int)-'A')
            break;
    }

    return x1 - x2;
}

int skip_atoi_m(const char **s)
{
    register int i = 0;
    while(isdigit_m(**s))
    {
        i = i*10 + *((*s)++) - '0';
    }
    return i;
}

#if 0
        typedef __builtin_va_list   __gnuc_va_list;
        typedef __gnuc_va_list      va_list;
        #define va_start(v,l)       __builtin_va_start(v,l)
        #define va_end(v)           __builtin_va_end(v)
        #define va_arg(v,l)         __builtin_va_arg(v,l)

#endif // 0

int vsscanf(const char * buf, const char * fmt, va_list args)
{
    const char *str = buf;
    char *next;
    int num = 0;
    int qualifier;
    int base;
    int field_width = -1;
    int is_sign = 0;

    while(*fmt && *str) {
        /* skip any white space in format */
        /* white space in format matchs any amount of
         * white space, including none, in the input.
         */
        if (isspace_m(*fmt)) {
            while (isspace_m(*fmt))
                ++fmt;
            while (isspace_m(*str))
                ++str;
        }

        /* anything that is not a conversion must match exactly */
        if (*fmt != '%' && *fmt) {
            if (*fmt++ != *str++)
                break;
            continue;
        }

        if (!*fmt)
            break;
        ++fmt;

        /* skip this conversion.
         * advance both strings to next white space
         */
        if (*fmt == '*') {
            while (!isspace_m(*fmt) && *fmt)
                fmt++;
            while (!isspace_m(*str) && *str)
                str++;
            continue;
        }

        /* get field width */
        if (isdigit_m(*fmt))
            field_width = skip_atoi_m(&fmt);

        /* get conversion qualifier */
        qualifier = -1;
        if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L' || *fmt == 'Z') {
            qualifier = *fmt;
            fmt++;
        }
        base = 10;
        is_sign = 0;

        if (!*fmt || !*str)
            break;

        switch(*fmt++) {
        case 'c':
        {
            char *s = (char *) va_arg(args,char*);
            if (field_width == -1)
                field_width = 1;
            do {
                *s++ = *str++;
            } while(field_width-- > 0 && *str);
            num++;
        }
        continue;
        case 's':
        {
            char *s = (char *) va_arg(args, char *);
            if(field_width == -1)
                field_width = 2147483647;//INT_MAX;
            /* first, skip leading white space in buffer */
            while (isspace_m(*str))
                str++;

            /* now copy until next white space */
            while (*str && !isspace_m(*str) && field_width--) {
                *s++ = *str++;
            }
            *s = '\0';
            num++;
        }
        continue;
        case 'n':
            /* return number of characters read so far */
        {
            int *i = (int *)va_arg(args,int*);
            *i = str - buf;
        }
        continue;
        case 'o':
            base = 8;
            break;
        case 'x':
        case 'X':
            base = 16;
            break;
        case 'd':
        case 'i':
            is_sign = 1;
        case 'u':
            break;
        case '%':
            /* looking for '%' in str */
            if (*str++ != '%')
                return num;
            continue;
        default:
            /* invalid format; stop here */
            return num;
        }

        /* have some sort of integer conversion.
         * first, skip white space in buffer.
         */
        while (isspace_m(*str))
            str++;

        if (!*str || !isdigit_m(*str))
            break;

        switch(qualifier) {
        case 'h':
            if (is_sign) {
                short *s = (short *) va_arg(args,short *);
                *s = (short) simple_strtol(str,&next,base);
            } else {
                unsigned short *s = (unsigned short *) va_arg(args, unsigned short *);
                *s = (unsigned short) simple_strtoul(str, &next, base);
            }
            break;
        case 'l':
            if (is_sign) {
                long *l = (long *) va_arg(args,long *);
                *l = simple_strtol(str,&next,base);
            } else {
                unsigned long *l = (unsigned long*) va_arg(args,unsigned long*);
                *l = simple_strtoul(str,&next,base);
            }
            break;
        case 'L':
            if (is_sign) {
                long long *l = (long long*) va_arg(args,long long *);
                *l = simple_strtoll(str,&next,base);
            } else {
                unsigned long long *l = (unsigned long long*) va_arg(args,unsigned long long*);
                *l = simple_strtoull(str,&next,base);
            }
            break;
        case 'Z':
        {
            unsigned long *s = (unsigned long*) va_arg(args,unsigned long*);
            *s = (unsigned long) simple_strtoul(str,&next,base);
        }
        break;
        default:
            if (is_sign) {
                int *i = (int *) va_arg(args, int*);
                *i = (int) simple_strtol(str,&next,base);
            } else {
                unsigned int *i = (unsigned int*) va_arg(args, unsigned int*);
                *i = (unsigned int) simple_strtoul(str,&next,base);
            }
            break;
        }
        num++;

        if (!next)
            break;
        str = next;
    }
    return num;
}

int sscanf_m(const char *buf,const char *fmt,...)
{
    va_list args;
    int i;

    va_start(args,fmt);
    i = vsscanf(buf,fmt,args);
    va_end(args);

    return i;
}

char *strstr_m(const char *s1,const char *s2)
{
    int l1,l2;
    l2 = strlen_m(s2);
    if (!l2)
        return (char *)s1;
    l1 = strlen_m(s1);
    while (l1 >= l2)
    {
        l1 --;
        if (!memcmp_m(s1, s2, l2))
            return (char *)s1;
        s1 ++;
    }

    return NULL;

}

unsigned long strlen_m(const char *s)
{
    const char *sc;
    for(sc = s; *sc != '\0'; ++sc)
        ;

    return sc - s;
}

unsigned long strnlen_m(const char *s,unsigned long maxlen)
{
    const char *sc;
    for(sc = s; *sc != '\0' && sc - s < maxlen; ++sc)
        ;

    return sc - s;
}

char *strcpy_m(char *dest,const char *src)
{
    return strncpy_m(dest,src,strlen_m(src) + 1);
}

char *strncpy_m(char *dest,const char *src,unsigned long n)
{
    if(n != 0)
    {
        char *d = dest;
        const char *s = src;
        do
        {
            if((*d++ = *s++) == 0)
            {
                while(--n != 0)
                    *d++ = 0;
                break;
            }
        }
        while(--n != 0);
    }

    return (dest);
}

unsigned long strlcpy_m(char *dst,const char *src,unsigned long count)
{
    register char *d = dst;
    register const char *s = src;
    register unsigned long n = count;

    if(n != 0 && --n != 0)
    {
        do
        {
            if((*d++ = *s++) == 0)
                break;
        }
        while(--n != 0);
    }

    if(n == 0)
    {
        if(count != 0)
            *d = '\0';
        while(*s++) ;
    }

    return (s - src - 1);
}

char *strncat_m(char *dest,const char *src,unsigned long count)
{
    char *tmp = dest;

    if(count)
    {
        while(*dest)
            dest++;

        //while((*dest++ = *src++))
				while(1)
        {
						*dest++ = *src++;
					
            if(--count == 0)
            {
                *dest = '\0';
                break;
            }
        }
    }

    return tmp;
}

char *strcat_m(char *dest,const char *src)
{
    char *tmp = dest;
    while(*dest)
        dest++;
    while((*dest++ = *src++) != '\0')
        ;

    return tmp;
}

char *strchr_m(const char *s1,int i)
{
    const unsigned char *s = (const unsigned char *)s1;
    unsigned char c = (unsigned int)i;

    while (*s && *s != c)
    {
        s++;
    }

    if (*s != c)
    {
        s = NULL;
    }

    return (char *) s;
}

char *strrchr_m(const char *t,int c)
{
    register char ch;
    register const char *l = 0;

    ch = c;
    for(;;)
    {
        if(*t == ch)
            l = t;

        if(!*t)
            return (char *)l;

        ++t;
    }

    //return (char *)l;
}

char *strtok_m(char *s,const char *delim)
{
    static char *strtok_pos;
    return strtok_r_m(s,delim,&strtok_pos);
}

char *strtok_r_m(char *s,const char *delim,char **ptrptr)
{
    char *tmp = 0;
    if(s == 0)
    {
        s = *ptrptr;
    }
    s += strspn_m(s,delim);
    if(*s)
    {
        tmp = s;
        s += strcspn_m(s,delim);

        if(*s)
        {
            *s++ = 0;
        }
    }
    *ptrptr = s;

    return tmp;
}

unsigned long strcspn_m(const char *s,const char *reject)
{
    unsigned int l = 0;
    int a = 1,i,al = strlen_m(reject);

    while((a) && (*s))
    {
        for(i = 0;(a) && (i < al);i++)
        {
            if(*s == reject[i])
            {
                a = 0;
            }
            if(a)
            {
                l++;
            }
            s++;
        }
    }

    return l;
}

unsigned long strspn_m(const char *s,const char *accept)
{
   unsigned int l = 0;
   int a = 1,i, al = strlen_m(accept);

   while((a) && (*s))
   {
       for(a = i = 0; (!a) && (i < al); i++)
       {
           if(*s == accept[i])
           {
                a = 1;
           }
       }
       if(a)
       {
           l++;
       }
       s++;
   }

   return l;
}
//将str转换为unsigned long
unsigned long simple_strtoul(const char *cp,char **endp,unsigned int base)
{
    unsigned long result = 0,value;

    if (!base) {
        base = 10;
        if (*cp == '0') {
            base = 8;
            cp++;
            if ((toupper_m(*cp) == 'X') && isxdigit_m(cp[1])) {
                cp++;
                base = 16;
            }
        }
    } else if (base == 16) {
        if (cp[0] == '0' && toupper_m(cp[1]) == 'X')
            cp += 2;
    }
    while (isxdigit_m(*cp) &&
           (value = isdigit_m(*cp) ? *cp-'0' : toupper_m(*cp)-'A'+10) < base) {
        result = result*base + value;
        cp++;
    }
    if (endp)
        *endp = (char *)cp;
    return result;
}
//将str转换为long
long simple_strtol(const char *cp,char **endp,unsigned int base)
{
    if(*cp=='-')
        return -simple_strtoul(cp+1,endp,base); //负值
    return simple_strtoul(cp,endp,base);
}

//将str转换为unsigned long long
unsigned long long simple_strtoull(const char *cp,char **endp,unsigned int base)
{
    unsigned long long result = 0, value;

    if (*cp == '0') {
        cp++;
        if ((toupper_m(*cp) == 'X') && isxdigit_m(cp[1])) {
            base = 16;
            cp++;
        }
        if (!base) {
            base = 8;
        }
    }
    if (!base) {
        base = 10;
    }
    while (isxdigit_m(*cp) && (value = isdigit_m(*cp)
                ? *cp - '0'
                : (islower_m(*cp) ? toupper_m(*cp) : *cp) - 'A' + 10) < base) {
        result = result * base + value;
        cp++;
    }
    if (endp)
        *endp = (char *) cp;
    return result;
}

long long simple_strtoll(const char *cp,char **endp,unsigned int base)
{
    if(*cp=='-')
        return -simple_strtoull(cp+1,endp,base);
    return simple_strtoull(cp,endp,base);
}

long strtol(const char *str,char **endptr,int base)
{
    return simple_strtol(str,endptr,base);
}

long long strtoll(const char *str,char **endptr,int base)
{
    return simple_strtoll(str,endptr,base);
}


int atoi(const char* s)
{
    long int v=0;
    int sign=1;
    while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) s++;
    switch (*s)
    {
    case '-':
        sign=-1;
    case '+':
        ++s;
    }
    while ((unsigned int) (*s - '0') < 10u)
    {
        v=v*10+*s-'0';
        ++s;
    }
    return sign==-1?-v:v;
}

long int atol_(const char* s)
{
    long int v=0;
    int sign=0;
    while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) ++s;
    switch (*s)
    {
        case '-': sign=-1;
        case '+': ++s;
    }
    while ((unsigned int) (*s - '0') < 10u)
    {
        v=v*10+*s-'0'; ++s;
    }
    return sign?-v:v;
}
