#include <stdbool.h>
#include "stm32f303xe.h"
#include "m24m01_EEPROM.h"
#include "delay.h"

#define NBYTE                  I2C_CR2_NBYTES_Pos	            //количество передаваемыи и получаемых байт (8 bit)
#define RELOAD                 24                               //Если NBYTE переданы 0 - прекратить 1 - продолжать передачу
#define SADD_BITS		        1					            //адресс slave устройства (7 bit)
#define RW_BIT		           10                    			//0 - запись, 1 - чтение
#define ADD10_BIT		       11                      		    //0 - 7бит адрес, 1 - 10 бит адрес
#define START_BIT		       13                      		    //0 - нет, 1 - старт, рестарт передачи
#define STOP_BIT		       14                      		    //0 - нет, 1 - стоп после передачи текущего байта
#define AUTOEND_BIT            25                       		//0 - softMode, 1 - autoEnd mode
#define NACK_BIT		        4					            //1 - нет ответа(устанавл аппаратно), сбрасывается программно I2C_ICR.NACKCF
#define RXNE_BIT		        2					            //1 - данные приняты
#define TXE_BIT		            0					            //1 - регистр пуст и готов для записи новых данных (запись единицы очищает регистр передачи)
#define TXIS_BIT                1                               //1 - регистр пуст и получен ответ ACK
#define NACKCF_BIT		        4					            //1 - сброс бита NACK
#define STOPCF_BIT		        5					            //1 - сброс стоп бита
#define TC_BIT                  6                               //1 - передача NBYTE завершена
#define TCR_BIT                 7                               //1 - передача NBYTE при RELOAD=1 завершена

#define TXRX_MAX                255U                            //Максимальное количество передаваемых/получаемых байт контроллером I2C за одну передачу
#define WAIT_TIME               1                               //Время ожидания ответа в милисекундах

#define SET_I2C_ADR(adr)        I2C->CR2 |= (*((uint8_t*)&adr + 2) << 1 | EEPROM_BASE)   //установка I2C адреса
#define SET_EEPROM_ADR(adr,x)   I2C->TXDR = (*((uint8_t*)&adr + x))                      //установка адреса внутри EEPROM
#define SEND_BYTES_NORELOAD(x)  I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE)                                 //установка кол-ва передаваемых принимаемых байт
#define SEND_BYTES_RELOAD(x)    I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE) | (1 << RELOAD)
#define START_COND()            I2C->CR2 |= (1 << START_BIT)
#define STOP_COND()             I2C->CR2 |= (1 << STOP_BIT)
//#define TRANSMIT_OK()           (I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) != 0


/*Для общения с I2C EEPROM необходимо указать адрес микросхемы на шине I2C(7 бит)
и адрес ячейки памяти внутри EEPROM(16 бит)
Переменная "adr" состоит: 
2-й байт - адрес на шине I2C,
1-й байт - старший байт адреса в EEPROM
0-й байт - младший байт адреса в EEPROM
*/

static inline uint32_t set_transmit_bytes(int32_t txBytes)
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

//bool wait_to_send()
//{
//    uint8_t time = ms_time();
//    while(1)
//    {
//        if (TRANSMIT_OK())
//        {
//            return true;
//        }
//        if ((ms_time() - time) ==  WAIT_TIME)
//        {
//            return false;
//        }
//    }
//}

bool wait_to_send()
{
    while((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) == 0)
    {
        if((I2C->ISR & (1 << NACKCF_BIT)) != 0)
        {
            return false;
        }
    }
    return true;
}

/********************************************************************/
//       Передача данных в шину i2c 
/********************************************************************/
eepromErr i2c_transmit(uint32_t adr, uint8_t* data, int32_t nBytes)
{
    //nBytes  - общее количество байт которое необходимо передать по шине I2C
    I2C->ISR |= (1 << TXE_BIT);                     //Очистка регистра передатчика
    I2C->CR2 = 0;                                   //Очистка регистра состояния
    I2C->ICR = (1 << NACKCF_BIT);                  //Сброс флага NACK
    nBytes += ADR_BYTES;                            //Установка общего количества передаваемых байт (адрес + данныые)
    
    uint32_t i = 0;                                 //Счётчик текущего передаваемого байта
    
    nBytes = set_transmit_bytes(nBytes);
    
    //*********** Установка адреса на шине I2C с внутренним адресом в EEPROM *********
    SET_I2C_ADR(adr);                                               //установка адреса EEPROM на шине I2C (2-й байт)
    
    
    START_COND();                                                   //Старт передачи данных
    if(!wait_to_send())
    {
        STOP_COND();
        return ERR_ADR;
    }
            
    SET_EEPROM_ADR(adr, 1);                                         //Установка старшего байта адреса в EEPROM (1-й байт)
    wait_to_send();                                                 //ожидание передачи данных
    //while((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) == 0);
    
    SET_EEPROM_ADR(adr, 0);                                         //установка младшего байта адреса
    wait_to_send();
    //while((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) == 0);

    //*********** Передача данных ***********        
    
    do
    {
        I2C->TXDR = data[i];
        wait_to_send();
        //while((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) == 0);
        
        if((I2C->ISR & (1 << TCR_BIT)) != 0)
        {
            nBytes = set_transmit_bytes(nBytes);  
        } 
        i++;
        
    }while(((I2C->ISR & (1 << TC_BIT)) == 0));
    STOP_COND();
    return OK;
}

/********************************************************************/
//       Приём данных из шины i2c 
/********************************************************************/
//uint8_t i2c_receive(uint32_t adr, uint8_t* data, uint32_t nBytes)
//{
//    uint32_t rxBytes;                                               //количество передаваемых байт за передачу (n < 255)
//    uint32_t i = 0;
//    I2C->CR2 = 0;                                                   //обнуляем регистр состояния_2 
//       
//    if(nBytes + ADR_BYTES < TXRX_MAX)
//    {
//        rxBytes = nBytes + ADR_BYTES;
//        nBytes = 0;
//    }
//    else
//    {
//        txBytes = TXRX_MAX;
//        nBytes -= txBytes;
//        I2C->CR2 |= (1 << RELOAD);
//    }
//    
//    I2C->ISR |= (1 << I2C_ISR_TXE);                                 //установка флага TXE позволяет очистить регистр передатчика
//    I2C->CR2 |= (txBytes << NBYTE) | (0 << RW_BIT);                 //Выбор режима передачи I2C
//    
//    //*********** Установка адреса на шине I2C с внутренним адресом в EEPROM *********
//    SET_I2C_ADR(adr);                                               //установка адреса EEPROM на шине I2C (2-й байт)
//    
//    START_COND();                                                   //Старт передачи данных
//    
//    WAIT_TO_SEND();
//    txBytes--;
//    
//    SET_EEPROM_ADR(adr, 1);                                         //Установка старшего байта адреса в EEPROM (1-й байт)
//    WAIT_TO_SEND();                                                 //ожидание передачи данных
//    txBytes--;
//    
//    SET_EEPROM_ADR(adr, 0);                                         //установка младшего байта адреса
//    WAIT_TO_SEND();
//    txBytes--;
//    
//    //*********** Передача данных ***********        
//    
//    while(1)
//    {
//        I2C->TXDR = data[i];
//        WAIT_TO_SEND();            
//        if((I2C->ISR & (1 << TC_BIT)) != 0)     //Конец передачи данных
//        {
//            asm("nop");
//            break;
//        }
//        if((I2C->ISR & (1 << TCR_BIT)) != 0)
//        {
//            if(nBytes < TX_MAX)
//            {
//                txBytes = nBytes;
//                nBytes = 0;
//                SEND_BYTES_NORELOAD(txBytes);
//            }
//            else
//            {
//                txBytes = TX_MAX;
//                nBytes -= txBytes;
//                SEND_BYTES_RELOAD(txBytes);
//            }
//        } 
//        i++;
//        txBytes--;            
//    }
//    STOP_COND();
//    return 0;
//}



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