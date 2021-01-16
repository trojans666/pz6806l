#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_iwdg.h"

#include "core_cm3.h"
#include "misc.h"

#include "bsp_include.h"
#include "bsp_target.h"

#define disableInterrupt()      __set_PRIMASK(1)        /*!< �����ж� */
#define enableInterrupt()       __set_PRIMASK(0)        /*!< �����ж� */

/**
    ʱ�ӳ�ʼ��
*/
static void bsp_rcc_init(void)
{
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    while(RCC_WaitForHSEStartUp() != SUCCESS)
    {
            ;
    }

    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE); /* SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div2); /* AHB */

    RCC_PCLK1Config(RCC_HCLK_Div1); /* APB1 */
    RCC_PCLK2Config(RCC_HCLK_Div1); /* APB2 */
}

/**
    GPIO��ʼ��
*/
static void bsp_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* LED GPIO INIT */
    RCC_APB2PeriphClockCmd(LED1_APB | LED2_APB | LED3_APB | LED4_APB | LED5_APB | LED6_APB | LED7_APB | LED8_APB,ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LED1_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LED2_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED3_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LED3_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED4_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LED4_PORT,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED5_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LED5_PORT,&GPIO_InitStructure);
}

static void bsp_timer_init(void)
{
    RCC_ClocksTypeDef rcc_clk;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_GetClocksFreq(&rcc_clk);

    /* systick  /1000 = 1ms /100 = 10ms*/
    SysTick_Config(rcc_clk.HCLK_Frequency / (1000000 / SYSTICK_TICK_US));
    /* generic timer */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; /*ʱ�Ӳ���Ƶ */
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/* ���ϼ��� */
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    /* T = (period + 1) * (prescaler + 1) / Tclk 8MHZ = 8000000hz(s)
       (100) * (8000000/100/1000*n) / 8000000 = n*8000 / 8000000 = n/1000(s)
       n = 1 1ms
       n = 2 2ms
    */
    TIM_TimeBaseInitStructure.TIM_Prescaler = rcc_clk.PCLK1_Frequency / 100 / 1000 * TIMER_TICK_MS - 1;/* ��Ƶϵ�� */
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;/* �������� */
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2,TIM_FLAG_Update); /* �������������ʱ�����ĸ��±�־ */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); /* ʹ�ܶ�ʱ���ж� */
    TIM_Cmd(TIM2,ENABLE); /* ʹ�ܶ�ʱ�� */
}

/* �ж����� */
static void bsp_nvic_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* �����ж������� (����ƫ��ʱ�� */

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    /* timer */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;/* ��ռ���ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* ��Ӧ���ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART1 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;/* ��ռ���ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5; /* ��Ӧ���ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void bsp_watchdog_init(void)
{
    /* �ڲ����Ź� �������Ź��� LSI�ṩʱ��*/
    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){ ; }
    /* ʹ��Ԥ��Ƶ�Ĵ���PR����װ�ؼĴ���PLR��д */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    /* Tout = prv / 40000 * rlv (s) = 16 / 40000 * 4095 = 1.638s */
    IWDG_SetPrescaler(IWDG_Prescaler_16);
    IWDG_SetReload(0xfff);
    //IWDG_ReloadCounter(); ι��
    /* PVU ���Ź�Ԥ��Ƶֵ���� RVU�����Ź���������װ��ֵ���� */
    while(IWDG_GetFlagStatus(IWDG_FLAG_PVU) == RESET) {;}
    IWDG_Enable();
}

void bsp_target_init(void)
{
		disableInterrupt();

		bsp_rcc_init();
		bsp_gpio_init();
		bsp_timer_init();
		bsp_nvic_init();
		bsp_usart_init();
		bsp_watchdog_init();
	
	
	
		enableInterrupt();
}
