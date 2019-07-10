#include "stm32f303xe.h"
#include <stdlib.h>
#include <stdbool.h>
#include "mcuInit.h"
#include "m24m01_EEPROM.h"
#include "delay.h"

#define CPU_CLK 8000000UL           //текущая частота процессора

bool butOn = false;
//volatile uint32_t sysTick = 0;



int main()
{
    mcu_init();
    delay_init(CPU_CLK);
    uint8_t data[768];
    
    for(uint16_t i = 0, j = 1; i < sizeof(data); i++, j++)
    {
        data[i] = j;
        if(j == 255) j = 0;
    }
    //i2c_transmit(0x00, data, 253);
    delay_ms(10);
    i2c_transmit(0x00, data, 254);
    delay_ms(6);
    i2c_transmit(0x00, data, 255);
    delay_ms(1);
    i2c_transmit(0x00, data, 256);
    delay_ms(1);
    i2c_transmit(0x00, data, 257);
    delay_ms(5);
    i2c_transmit(0x00, data, 515);
    
    
    while(true)
    {
        if(butOn)
        {
            delay_ms(10);
            GPIOA->ODR |= LED_GREEN; 
            butOn = false;
            delay_ms(5000);
            GPIOA->ODR &= ~LED_GREEN;
        }
        
        
    }
}

//Обработчик внешнего прерывания (от кнопки)
void EXTI15_10_IRQHandler()
{
    butOn = true;
    EXTI->PR |= BUTTON; //сброс флага прерывания
}
