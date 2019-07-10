#include <stdint.h>
#include "delay.h"

volatile uint16_t ms_tick = 0;

//Функция задержки тактируемая системным таймером
void delay_ms(uint16_t milliseconds)
{
    uint16_t start = ms_tick;
    while((ms_tick - start) < milliseconds);
}

uint16_t ms_time()
{
     return ms_tick;
}

//Настройка SysTimer
void delay_init(uint32_t cpuFrequency)
{
    *(uint32_t*)0xE000E014 = cpuFrequency / 1000 - 1;           //Загрузка значения в таймер 1мс
    *(uint32_t*)0xE000E018 = cpuFrequency / 1000 - 1;           //Устанавливаем текущее значение таймера 1мс
    *(uint32_t*)0xE000E010 = (1 << 2) | (1 << 1) | (1 << 0);    //(тактировать от процессора)|(разрешить прерывание)|(включить таймер)
}

//Обработчик прерывания от системного таймера
void SysTick_Handler()
{
    ms_tick++;
}