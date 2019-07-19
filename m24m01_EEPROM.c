#include <stdbool.h>
#include "stm32f303xe.h"
#include "m24m01_EEPROM.h"
#include "delay.h"

#define NBYTE                   I2C_CR2_NBYTES_Pos	         //Количество передаваемыи и получаемых байт (до 255 байт)
#define RELOAD                  I2C_CR2_RELOAD_Pos               //Если NBYTE переданы 0 - прекратить 1 - продолжать передачу
#define RW_BIT		            I2C_CR2_RD_WRN_Pos         	     //1 - чтение, 0 - запись
#define START_BIT		        I2C_CR2_START_Pos                //1 - старт, рестарт передачи
#define STOP_BIT		        I2C_CR2_STOP_Pos             	 //1 - стоп после передачи текущего байта
#define AUTOEND_BIT             I2C_CR2_AUTOEND_Pos              //1 - autoEnd mode, 0 - softMode 
#define NACK_BIT		        I2C_CR2_NACK_Pos		 //1 - нет ответа(устанавл аппаратно), сбрасывается программно I2C_ICR.NACKCF
#define RXNE_BIT		        I2C_ISR_RXNE_Pos		 //1 - данные приняты
#define TXE_BIT		            I2C_ISR_TXE_Pos			 //1 - регистр пуст и готов для записи новых данных (запись единицы очищает регистр передачи)
#define TXIS_BIT                I2C_ISR_TXIS_Pos                 //1 - регистр пуст и получен ответ ACK
#define NACKCF_BIT		        I2C_ISR_NACKF_Pos		 //1 - сброс бита NACK
#define STOPCF_BIT		        I2C_ISR_STOPF_Pos		 //1 - сброс стоп бита
#define TC_BIT                  I2C_ISR_TC_Pos                   //1 - передача NBYTE завершена
#define TCR_BIT                 I2C_ISR_TCR_Pos                  //1 - передача NBYTE при RELOAD=1 завершена


#define TXRX_MAX                255U                            //Максимальное количество передаваемых/получаемых байт контроллером I2C за одну передачу

#define SET_I2C_ADR(adr)        I2C->CR2 |= (*((uint8_t*)&adr + 2) << 1 | EEPROM_BASE)   //установка I2C адреса
#define SET_EEPROM_ADR(adr,x)   I2C->TXDR = (*((uint8_t*)&adr + x))                      //установка адреса внутри EEPROM
#define SEND_BYTES_NORELOAD(x)  I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE)                                 //установка кол-ва передаваемых принимаемых байт
#define SEND_BYTES_RELOAD(x)    I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE) | (1 << RELOAD)
#define START_COND()            I2C->CR2 |= (1 << START_BIT)
#define STOP_COND()             I2C->CR2 |= (1 << STOP_BIT)
#define SET_WR()                I2C->CR2 &= (1 << RW_BIT)
#define SET_RD()                I2C->CR2 |= (1 << RW_BIT)
#define TRANSMIT_OK()           ((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) != 0)


/*Для общения с I2C EEPROM необходимо указать адрес микросхемы на шине I2C(7 бит)
и адрес ячейки памяти внутри EEPROM(16 бит)
Переменная "adr" состоит: 
2-й байт - адрес на шине I2C,
1-й байт - старший байт адреса в EEPROM
0-й байт - младший байт адреса в EEPROM
*/

static uint32_t set_transmit_mode(int32_t txBytes)
{
    if(txBytes <= TXRX_MAX)
    {
        SEND_BYTES_NORELOAD(txBytes);
        txBytes = 0;
    }
    else
    {
        SEND_BYTES_RELOAD(TXRX_MAX);
        txBytes -= TXRX_MAX;
    }
    return txBytes;
}

static bool wait_to_send()
{
    bool result = true;
    uint32_t isr;
    while(1)
    {
        isr = I2C->ISR;
        if((isr & (1 << NACKCF_BIT)) != 0)
        {
            result = false;
            break;
        }
        if((isr & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) != 0)
        {
            result = true;
            break;
        }
    }
    
    return result;
}

/********************************************************************/
//       Передача данных в шину i2c 
/********************************************************************/
//eepromErr i2c_transmit(uint32_t adr, uint8_t* data, int16_t nBytes)
//{
//    eepromErr result = OK;
//    
//    //nBytes  - общее количество байт которое необходимо передать по шине I2C
//    I2C->ISR |= (1 << TXE_BIT);                     //Очистка регистра передатчика
//    nBytes += ADR_BYTES;                            //Установка общего количества передаваемых байт (адрес + данныые)
//    
//    uint32_t i = 0;                                 //Счётчик текущего передаваемого байта
//    
//    nBytes = set_transmit_mode(nBytes);
//    
//    //*********** Установка адреса на шине I2C с внутренним адресом в EEPROM *********
//    SET_I2C_ADR(adr);                                               //установка адреса EEPROM на шине I2C (2-й байт)
//    
//    
//    START_COND();                                                   //Старт передачи данных
//    if(!wait_to_send())
//    {
//        result = ERR_ADR;
//        goto end_tr;        
//    }
//    
//    SET_EEPROM_ADR(adr, 1);                                         //Установка старшего байта адреса в EEPROM (1-й байт)
//    wait_to_send();                                                 //ожидание передачи данных
//    
//    SET_EEPROM_ADR(adr, 0);                                         //установка младшего байта адреса
//    wait_to_send();
//    
//    //*********** Передача данных ***********        
//    do
//    {
//        I2C->TXDR = data[i++];
//        if(!wait_to_send())
//        {
//            result = ERR_WRITE;
//            break;
//        }
//               
//        if((I2C->ISR & (1 << TCR_BIT)) != 0)
//        {
//            nBytes = set_transmit_mode(nBytes);  
//        }         
//    }while(((I2C->ISR & (1 << TC_BIT)) == 0));
//
//end_tr:   
//    STOP_COND();
//    return result;
//}

/********************************************************************/
//       Приём/передача данных из/в шину i2c 
/********************************************************************/
eepromErr i2c_rw(uint32_t adr, uint8_t* data, uint16_t nBytes, eepromDir dir)
{
    //nBytes  - общее количество байт которое необходимо передать по шине I2C
    I2C->ISR |= (1 << TXE_BIT);                         //Очистка регистра передатчика
    I2C->CR2 &= ~(1 << RW_BIT);                         //Режим записи I2C
    I2C->ICR = (1 << NACKCF_BIT);                       //Сброс NACK флага
    I2C->ICR = (1 << STOPCF_BIT);                       //Сброс STOP флага
    
    uint32_t i = 0;                                     //Счётчик текущего передаваемого байта
            
    if(dir == WR)
    {
        nBytes += ADR_BYTES;
        nBytes = set_transmit_mode(nBytes);
    }
    else if(dir == RD)
    {
        set_transmit_mode(ADR_BYTES);
    }
    
    //******* Установка полного адреса (I2C адрес + EEPROM адрес) *********
    SET_I2C_ADR(adr);                               //установка адреса EEPROM на шине I2C (2-й байт)
    START_COND();                                   //Старт передачи данных
    if(!wait_to_send())
    {
        return ERR_ADR;        
    }
    
    SET_EEPROM_ADR(adr, 1);                         //Установка старшего байта адреса в EEPROM (1-й байт)
    wait_to_send();                                 //ожидание передачи данных
    
    SET_EEPROM_ADR(adr, 0);                         //установка младшего байта адреса
    wait_to_send();
    
    //*********** Передача данных ***********
    if(dir == WR)
    {
        while((I2C->ISR & (1 << TC_BIT)) == 0)
        {
            I2C->TXDR = data[i++];
            if(!wait_to_send())
            {
                STOP_COND();
                return ERR_WRITE;
            }
            
            if((I2C->ISR & (1 << TCR_BIT)) != 0)
            {
                nBytes = set_transmit_mode(nBytes);  
            }         
        }
    }  
    
    //*********** Приём данных ***********
    else if(dir == RD)
    {
        nBytes = set_transmit_mode(nBytes);
        I2C->CR2 |= (1 << RW_BIT);
        START_COND();
        while((I2C->ISR & (1 << TC_BIT)) == 0)
        {
            while((I2C->ISR & ((1 << RXNE_BIT) | ( 1 << TC_BIT))) == 0)
            {
                asm("nop");
            }
           
            data[i++] = I2C->RXDR;
            
            if((I2C->ISR & (1 << TCR_BIT)) != 0)
            {
                nBytes = set_transmit_mode(nBytes);  
            }         
        }
    }  
    STOP_COND();      
    return OK;
}


eepromErr eeprom_write(uint32_t adr, uint8_t* data, uint16_t count)
{
    return OK;
}

eepromErr eeprom_read(uint32_t adr, uint8_t* data, uint16_t count)
{
    return OK;
}

eepromErr eeprom_verifi(uint32_t addr, uint8_t* data, uint16_t count)
{
    
    return OK;
}