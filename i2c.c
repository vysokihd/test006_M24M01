#include <stdbool.h>
#include "stm32f303xe.h"
#include "i2c.h"


#define NBYTE                   I2C_CR2_NBYTES_Pos	            //Количество передаваемыи и получаемых байт (до 255 байт)
#define RELOAD                  I2C_CR2_RELOAD_Pos              //Если NBYTE переданы 0 - прекратить 1 - продолжать передачу
#define RW_BIT		            I2C_CR2_RD_WRN_Pos         	    //1 - чтение, 0 - запись
#define START_BIT		        I2C_CR2_START_Pos               //1 - старт, рестарт передачи
#define STOP_BIT		        I2C_CR2_STOP_Pos             	//1 - стоп после передачи текущего байта
#define AUTOEND_BIT             I2C_CR2_AUTOEND_Pos             //1 - autoEnd mode, 0 - softMode 
#define NACK_BIT		        I2C_CR2_NACK_Pos		        //1 - нет ответа(устанавл аппаратно), сбрасывается программно I2C_ICR.NACKCF
#define RXNE_BIT		        I2C_ISR_RXNE_Pos		        //1 - данные приняты
#define TXE_BIT		            I2C_ISR_TXE_Pos			        //1 - регистр пуст и готов для записи новых данных (запись единицы очищает регистр передачи)
#define TXIS_BIT                I2C_ISR_TXIS_Pos                //1 - регистр пуст и получен ответ ACK
#define NACKCF_BIT		        I2C_ISR_NACKF_Pos		        //1 - сброс бита NACK
#define STOPCF_BIT		        I2C_ISR_STOPF_Pos		        //1 - сброс стоп бита
#define TC_BIT                  I2C_ISR_TC_Pos                  //1 - передача NBYTE завершена
#define TCR_BIT                 I2C_ISR_TCR_Pos                 //1 - передача NBYTE при RELOAD=1 завершена
#define BUSY_BIT                I2C_ISR_BUSY_Pos            //1 - шина занята, 0 - шина свободна

#define TXRX_MAX                255U                            //Максимальное количество передаваемых/получаемых байт контроллером I2C за одну передачу

#define SET_I2C_DEV(adr)        I2C->CR2 = ((~(0xFE) & I2C->CR2) | dev)                  //установка I2C адреса
#define SET_EEPROM_ADR(adr,x)   I2C->TXDR = (*((uint8_t*)&adr + x))                      //установка адреса внутри EEPROM
#define SEND_BYTES_NORELOAD(x)  I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE)                                 //установка кол-ва передаваемых принимаемых байт
#define SEND_BYTES_RELOAD(x)    I2C->CR2 = (~(0x1FF << NBYTE) & I2C->CR2) | (x << NBYTE) | (1 << RELOAD)
#define START_COND()            I2C->CR2 |= (1 << START_BIT)
#define STOP_COND()             I2C->CR2 |= (1 << STOP_BIT)
#define SET_WR()                I2C->CR2 &= ~(1 << RW_BIT)
#define SET_RD()                I2C->CR2 |= (1 << RW_BIT)
#define TRANSMIT_OK()           ((I2C->ISR & (1 << TXIS_BIT | 1 << TCR_BIT | 1 << TC_BIT)) != 0)
#define NACKF()                 ((I2C->ISR & (1 << NACK_BIT)) != 0)
#define TRANSFER_COMPL()        ((I2C->ISR & (1 << TC_BIT)) != 0)
#define TRANSFER_COMPL_REL()    ((I2C->ISR & (1 << TCR_BIT)) != 0)
#define BUSY()                  ((I2C->ISR & (1 << BUSY_BIT)) != 0)


//static uint8_t i2c_dev;     //Адрес устройства на шине i2c (7..1 бит) + чтение-запись(0 бит)
static uint16_t nBytes;     //Количество передаваемых и получаемых байт по шине i2c
//static uint8_t* data;       //Указатель на массив чтения и записи
//static bool busy;           //true - шина занята, false - шина свободна

static void set_transmit_bytes(int16_t bytes);
static bool wait_to_send();


//******** Выбор устройства на шине I2C ***************** 
void I2C_device_select(uint8_t dev)
{
    I2C->ICR = (1 << NACKCF_BIT);               //Сброс NACK флага
    I2C->ICR = (1 << STOPCF_BIT);               //Сброс STOP флага
    I2C->ISR = (1 << TXE_BIT);                  //Очистка регитра передатчика
    //i2c_dev = dev;                            //Адрес устройства на шине i2c
    SET_I2C_DEV(dev);                           //Выбор устройства на шине I2C
    return;
}


////********* Подготовка шины i2c к передаче дынны ********************/
//void I2C_start(size)
//{
//    nBytes = size;
//}

//******** Передача данных в шину i2c данными по шине I2C *******************************/
i2c_status I2C_write(uint8_t* data, uint16_t size)
{
    if(size == 0) return I2C_NODATA;
    //if(BUSY()) return I2C_BUSY;
    
    uint16_t item = 0;                        //Счётчик текущего передаваемого байта
    nBytes = size;
       
    SET_WR();                                 //Установка режима записи
    I2C->ISR = (1 << TXE_BIT);                //Очистка регитра передатчика
    set_transmit_bytes(nBytes);               //Установка количества передаваемых байт за фрейм
    START_COND();                             //Начать передачу данных
    
    if(!wait_to_send())                       //Ожидание отправки адресной посылки
    {
        //busy = false;
        return I2C_NACKF;                     //Ошибка, не получен ответ.
    }
    
    //Передача данных в шину I2C побайтно
    while(1)
    {
        I2C->TXDR = data[item++];
        if(!wait_to_send())                       //Ожидание отправки адресной посылки
        {
            return I2C_ERR;                       //Ошибка, не получен ответ.
        }
        if(TRANSFER_COMPL_REL())
        {
            set_transmit_bytes(nBytes);
        }
        if(TRANSFER_COMPL()) break;
    }
    return I2C_OK;
}


//********  *****************
i2c_status i2c_read(uint8_t* data, uint16_t nBytes)
{
    //nBytes - общее количество принимаемых байт
    uint16_t item = 0;                        //Счётчик текущего принимаемого байта
    uint8_t rx;                               //Кол-во принимаемых байт во фрэйме
    I2C->ISR = (1 << TXE_BIT);                //Очистка регитра передатчика
    
    if(nBytes == 0) return I2C_NODATA;
   
    set_transmit_bytes(nBytes);          //Установка количества принимаемых байт
            
    /*if(mode.START == 1)*/ START_COND();
    
    if(!wait_to_send())                       //Ожидание отправки адресной посылки
    {
        return I2C_NACKF;                     //Ошибка, не получен ответ.
    }
    
    //Передача данных в шину I2C побайтно
    while(nBytes > 0)
    {
        for(; rx > 0; rx--, nBytes--, item++)
        {
            while((I2C->ISR & (1 << RXNE_BIT)) == 0);
            data[item] = I2C->RXDR;
        }
        if(TRANSFER_COMPL_REL())
        {
            set_transmit_bytes(nBytes);
        }
    }

    /*if(mode.STOP == 1)*/ STOP_COND();      
    
    return I2C_OK;
}




static void set_transmit_bytes(int16_t bytes)
{
    //uint32_t cr2 = I2C->CR2;
    //cr2 &= ~((0xFF << NBYTE) | (1 << RELOAD));
                
    if(bytes <= TXRX_MAX)
    {
        //cr2 |= (txBytes << NBYTE);
        SEND_BYTES_NORELOAD(bytes);
        nBytes = 0;
    }
    else
    {
        //cr2 |= (TXRX_MAX << NBYTE) | (1 << RELOAD);
        SEND_BYTES_RELOAD(TXRX_MAX);
        nBytes -= TXRX_MAX;
    }
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

void I2C_stop()
{
    STOP_COND();
    //busy = false;
}

