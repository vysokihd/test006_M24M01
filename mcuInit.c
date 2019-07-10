#include <stdint.h>
#include "mcuInit.h"
#include "stm32f303xe.h"

void mcu_init()
{
    //Включаем LED порт
    RCC-> AHBENR |= RCC_AHBENR_GPIOAEN;             //вкл. тактировние порта A
    GPIOA->MODER |= GPIO_MODER_MODER5_0;            //PA5 - выход
    
    //Включаем порт Кнопки
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;              //вкл. тактировние порта С
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR13_0;           //PC13 PullUp
    
    //включаем порт для работы I2C
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;              //включаем тактирование порта B
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0;            //Подтяжка к питанию PB8
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR9_0;            //Подтяжка к питанию PB9
    GPIOB->MODER |= GPIO_MODER_MODER8_1;            //PB8 в альтернативную функцию
    GPIOB->MODER |= GPIO_MODER_MODER9_1;            //PB9 в альтернативную функцию
    GPIOB->AFR[1] |= (1 << 2) | (1 << 6);           //Выбор PB8 - SCL, PB9 - SDA
        
    //Подключаем EXTI к PC13
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	        //вкл. System configur controller
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;  //подключить PC13(кнопку) к EXTI13
    EXTI->IMR |= BUTTON;                            //включаем прерываение от EXTI13
    EXTI->FTSR |= BUTTON;                           //прерываение по фронту
    //EXTI->RTSR |= BUTTON;                         //прерываение по спаду
      
    //Включаем и настраиваем I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;             //вкл. тактирование I2C1
    I2C1->TIMINGR = 0x2000090E;                     //конфигурирование таймингов
    I2C1->CR1 |= I2C_CR1_PE;                        //утсановка бита PE (включаем шину)
    
    NVIC_EnableIRQ(EXTI15_10_IRQn);                 //глобально разрешаем прерывание от EXTI10..15    
      
}