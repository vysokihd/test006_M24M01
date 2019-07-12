#pragma once

#define I2C I2C1                    //Выбор I2C модуля

typedef enum
{
    I2C_OK,
    I2C_NACKF,
        
}i2c_err;


typedef struct
{
    uint8_t STOP:1;             //1- формировать, 0 - не формировать СТОП
    uint8_t NOSTART:1;          //0 - формировать, 1 - не формировать СТАРТ
    uint8_t RW:1;               //1 - запись, 0 - чтение
    
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
i2c_err i2c_device_select(uint8_t dev, i2c_mode mode);

//******** Передача данных в шину I2C *******************************
i2c_err i2c_transmit(uint8_t* data, uint16_t count, i2c_mode mode);

//******** Чтение данных из шины I2C ********************************
i2c_err i2c_receive(uint8_t* data, uint16_t count, i2c_mode mode);