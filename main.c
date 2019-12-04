#include "stm32f303xe.h"
#include <stdlib.h>
#include <stdbool.h>
#include "mcuInit.h"
#include "m24m01_EEPROM.h"
#include "delay.h"
#include "i2c.h"

#define CPU_CLK 8000000UL           //текущая частота процессора

bool butOn = false;
//volatile uint32_t sysTick = 0;



int main()
{
    mcu_init();
    delay_init(CPU_CLK);
    uint8_t data[768];
    uint8_t recive[768] = {0};
    
    for(uint16_t i = 0, j = 0; i < sizeof(data); i++)
    {
        data[i] = j++;
        if(j == 256) j = 0;
    }
    
    I2C_device_select(0xA0);
    I2C_write(data, 10);
    I2C_stop();
   
    
    
    
    
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
