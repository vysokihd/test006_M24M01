#pragma once
//#ifndef _DELAY_H
//#define _DELAY_H


void delay_init(uint32_t cpuFrequency);
void delay_ms(uint16_t milliseconds);
uint16_t ms_time();

//#endif