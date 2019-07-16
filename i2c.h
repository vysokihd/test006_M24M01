#pragma once

#define I2C I2C1                    //Выбор I2C модуля

typedef enum
{
    I2C_TX_OK,
    I2C_RX_OK,
    I2C_NACKF,
    I2C_NODATA,
    I2C_ERR_TX,
    I2C_ERR_RX
        
}i2c_status;


typedef struct
{
    uint8_t STOP:1;             //1- формировать, 0 - не формировать СТОП
    uint8_t START:1;            //1 - формировать, 0 - не формировать СТАРТ
    uint8_t RW:1;               //1 - запись, 0 - чтение
    //uint8_t RESTART:1;          //1 - рестарт нужен, 0 - рестарт не нужем после переданных nByte
    
}i2c_mode;

/********************************************************************/
//       Функции для работы с I2C
/********************************************************************/
/*dev   - адрес устройства на шине I2C
  mode  - режим работы (старт, стоп, чтение/запись)
  data  - массив данных для передачи/получения
  count - количество данных (байт)
*/

//******** Формирование адресной посылки в шину I2C ***************** 
void i2c_device_select(uint8_t dev, i2c_mode mode);

//******** Передача данных в шину I2C *******************************
uint8_t i2c_transmit(uint8_t* data, uint16_t nBytes, i2c_mode mode);

//******** Чтение данных из шины I2C ********************************
uint8_t i2c_receive(uint8_t* data, uint16_t nBytes, i2c_mode mode);
