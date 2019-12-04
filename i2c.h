#pragma once

#define I2C I2C1                    //Выбор I2C модуля

typedef enum
{
    I2C_OK,
    I2C_NACKF,
    I2C_NODATA,
    I2C_ERR,
    I2C_BUSY,
        
}i2c_status;

typedef enum
{
    I2C_WRITE,
    I2C_READ,
}i2c_mode;

//******** Задание адреса устройства на шине I2C ********************/
void I2C_device_select(uint8_t dev);

////********* Подготовка шины i2c к предаче данных *******************/
//void I2C_start(uint16_t size);

//******** Передача данных в шину i2c данными по шине I2C ************/
i2c_status I2C_write(uint8_t* data, uint16_t size);

//******** Чтение данных по i2c
i2c_status I2C_read(uint8_t* data, uint16_t size);

//******** Формирование СТОП ******************************************/
void I2C_stop();

////******** Чтение данных из шины I2C ********************************/
//i2c_status I2C_receive(uint8_t* data, uint16_t nBytes);
