#ifndef __BSP_TARGET_H
#define __BSP_TARGET_H

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"



/*!< LED1   PC0 */
#define LED1_APB        RCC_APB2Periph_GPIOC
#define LED1_PORT       GPIOC
#define LED1_PIN        GPIO_Pin_0

/*!< LED2   PC1 */
#define LED2_APB        RCC_APB2Periph_GPIOC
#define LED2_PORT       GPIOC
#define LED2_PIN        GPIO_Pin_1

/*!< LED3   PC2 */
#define LED3_APB        RCC_APB2Periph_GPIOC
#define LED3_PORT       GPIOC
#define LED3_PIN        GPIO_Pin_2

/*!< LED4   PC3 */
#define LED4_APB        RCC_APB2Periph_GPIOC
#define LED4_PORT       GPIOC
#define LED4_PIN        GPIO_Pin_3

/*!< LED5   PC4 */
#define LED5_APB        RCC_APB2Periph_GPIOC
#define LED5_PORT       GPIOC
#define LED5_PIN        GPIO_Pin_4

/*!< LED6   PC5 */
#define LED6_APB        RCC_APB2Periph_GPIOC
#define LED6_PORT       GPIOC
#define LED6_PIN        GPIO_Pin_5

/*!< LED7   PC6 */
#define LED7_APB        RCC_APB2Periph_GPIOC
#define LED7_PORT       GPIOC
#define LED7_PIN        GPIO_Pin_6

/*!< LED8   PC7 */
#define LED8_APB        RCC_APB2Periph_GPIOC
#define LED8_PORT       GPIOC
#define LED8_PIN        GPIO_Pin_7




void bsp_target_init(void);

#endif // __BSP_TARGET_H
