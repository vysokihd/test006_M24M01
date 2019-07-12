#include <stdbool.h>
#include "stm32f303xe.h"
#include "i2c.h"


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

#define SET_I2C_ADR(adr)        I2C->CR2 |= ((~(0x3FF) & I2C->CR2) | adr)   //установка I2C адреса
#define SET_EEPROM_ADR(adr,x)   I2C->TXDR = (*((uint8_t*)&adr + x))                      //установка адреса внутри EEPROM
#define SEND_BYTES_NORELOAD(x)  I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE)                                 //установка кол-ва передаваемых принимаемых байт
#define SEND_BYTES_RELOAD(x)    I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE) | (1 << RELOAD)
#define START_COND()            I2C->CR2 |= (1 << START_BIT)
#define STOP_COND()             I2C->CR2 |= (1 << STOP_BIT)
#define SET_WR()                I2C->CR2 &= (1 << RW_BIT)
#define SET_RD()                I2C->CR2 |= (1 << RW_BIT)
#define TRANSMIT_OK()           ((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) != 0)
#define NACKF()                 (I2C->ISR & (1 << NACK_BIT) != 0)

//******** Формирование адресной посылки в шину I2C ***************** 
i2c_err i2c_device_select(uint8_t dev, i2c_mode mode)
{
    I2C->CR2 &= ~(0xf << NBYTE);    //Сброс количества передаваемых байт
    I2C->ICR = (1 << NACKCF_BIT);   //Сброс NACK флага
    I2C->ICR = (1 << STOPCF_BIT);   //Сброс STOP флага
    I2C->ISR = (1 << TXE_BIT);      //Очистка регитра передатчика
    //SET_I2C_ADR(dev);               //Выбор устройства на шине I2C
    I2C->CR2 |= ((~(0x3FF) & I2C->CR2) | dev);
    uint8_t result = I2C_OK;
    
    if(mode.RW) SET_RD();           //Установка режима чтение или запись
    else SET_WR();    
    
    START_COND();
    if(NACKF()) result = I2C_NACKF;
    
    if(mode.STOP || result == I2C_NACKF) STOP_COND();
    
    return result;
}

//******** Формирование адресной посылки в шину I2C *****************
i2c_err i2c_transmit(uint8_t* data, uint16_t count, i2c_mode mode)
{
    return I2C_OK;
}

//******** Формирование адресной посылки в шину I2C *****************
i2c_err i2c_receive(uint8_t* data, uint16_t count, i2c_mode mode)
{
    return I2C_OK;
}
