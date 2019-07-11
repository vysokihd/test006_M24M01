#define EEPROM_END              0x1ffff                         //Конечный адрес внутри EEPROM
#define EEPROM_BASE             0xA0                            //Базовый адрес EEPROM на шине I2C (0b1010xxxx)
#define DELAY_WRITE             4                               //Продолжительноть цикла внутренней записи
#define PAGE_SIZE               256                             //Размер старницы 
#define ADR_BYTES               2                               //Кол-во адресных байт внутри EEPROM

#define I2C I2C1                                                //Выбор I2C

typedef enum
{
    OK,             //успешно завершено
    ERR_READ,       //ошибка чтения
    ERR_WRITE,      //ошибка записи
    ERR_VER,        //ошибка верификации данных
    ERR_ADR,        //ошибка адреса
    
}eepromErr;

typedef enum
{
  WR,  
  RD    
}eepromDir;


//Запись/чтение/верификация EEPROM начиная с адреса "adr", из(в) массив "data" и колличеством байт "count"  
eepromErr eeprom_write(uint32_t adr, uint8_t* data, uint16_t count); 
eepromErr eeprom_read(uint32_t adr, uint8_t* data, uint16_t count);
eepromErr eeprom_verifi(uint32_t adr, uint8_t* data, uint16_t count);
//eepromErr i2c_transmit(uint32_t adr, uint8_t* data, int16_t nBytes);
eepromErr i2c_rw(uint32_t adr, uint8_t* data, uint16_t nBytes, eepromDir rw);