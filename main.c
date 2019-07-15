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
    i2c_mode mode = {.STOP = 0, .RW = 1, .START = 1};
    //mode.RW = 0;
    //mode.STOP = 1;
    //mode.NOSTART = 1;
    i2c_device_select(0x08, mode);
    i2c_device_select(0xA0, mode);
    i2c_device_select(0xB2, mode);
    i2c_device_select(0xC4, mode);
    //i2c_device_select(0xA0, );
    
//    i2c_rw(0x00, data, 256, WR);
//    delay_ms(1);
//    i2c_rw(0x100, data, 256, WR);
//    delay_ms(5);
//    i2c_rw(0x00, recive, 256, RD);
//    delay_ms(5);
//    i2c_rw(0x100, recive, 256, RD);
//    delay_ms(5);
//    i2c_rw(0x00, recive, 512, RD);
    
//    delay_ms(10);
//    i2c_transmit(0x00, data, 254);
//    delay_ms(6);
//    i2c_transmit(0x00, data, 255);
//    delay_ms(1);
//    i2c_transmit(0x00, data, 256);
//    delay_ms(1);
//    i2c_transmit(0x00, data, 257);
//    delay_ms(5);
//    i2c_transmit(0x00, data, 515);
    
    
    
    
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
