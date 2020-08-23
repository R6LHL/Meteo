
#include "headers.h"
///////////////////////////////////////////////////////
/* SYSTEM ERROR CODES
 *  0 - system ok
 *  1 - external sensor type is wrong
 *  2 - external sensor CRC is wrong
 */
////////////////////////////////////////////////////////
//variables
#if INTERNAL_SENSOR == BME280
  Adafruit_BME280 meteo_sensor;
  Serial_measure _temp_int;
  Serial_measure _press;
  Serial_measure _hum;
#endif

#if LCD_TYPE == LCD1602
  LiquidCrystal_I2C lcd(LCD_INITIAL_CODE,LCD_CHARS,LCD_STRINGS);  // Устанавливаем дисплей
#endif

#ifdef EXTERNAL_SENSOR
  #if EXTERNAL_SENSOR == DS18B20
    OneWire ext_temp_sens(EXTERNAL_SENSOR_PIN);
    byte ext_temp_addr[8];
    byte ext_temp_data[BYTE_ITERATOR];
    byte HighByte;
    byte LowByte; 
    float external_temperature;
    byte tempInt; 
    byte tempFloat;
    Serial_measure _temp_ext;
  #endif
#endif



byte system_error_code = 0;
bool device_sleep = false;

///////////////////////////////////////////////////////
//Tasks

#include "tasks.h"

///////////////////////////////////////////////////////
//interrupt handlers

#if UART_ENABLED == 1 //UART use TIMER0 so we use TIMER2 if UART is ENABLED
ISR(TIMER2_OVF_vect)
{
  //Serial.println("Interrupt is working");
  TaskManager::TimerTaskService_();
}
#else
ISR(TIMER0_OVF_vect)
{
  //Serial.println("Interrupt is working");
  TaskManager::TimerTaskService_();
}
#endif

ISR(INT0_vect)
{
  PCICR &= ~(1<<PCIE0);
  EIMSK &= ~(1<<INT0);

  TaskManager::SetTask_(device_wake,0);
}

void setup()
{
  noInterrupts();
  
/////////////////////////////////////
#if UART_ENABLED == 1 //UART use TIMER0 so we use TIMER2 if UART is ENABLED
  TCCR2B |= (1<<CS22); // (clk/64)
  TIMSK2 |= (1<<TOIE2); // ovf interrupt enabled
#else
 TCCR0B |= (1<<CS02); // (clk/64)
 TIMSK0 |= (1<<TOIE0); // ovf interrupt enabled
#endif

///////////////////////////////////  
#ifdef EXTERNAL_SENSOR
  #if EXTERNAL_SENSOR == DS18B20
    ext_temp_sens.search(ext_temp_addr); //searching address of ds18b20
    if (ext_temp_addr[0] == 0x28) //check sensor type
    {
      ext_temp_sens.reset();
      ext_temp_sens.select(ext_temp_addr);
     // ext_temp_sens.write(0x44,0); // start conversion with full power supply
    }
    else {system_error_code = 1;}
  #endif
#endif

//////////////////////////////////
  interrupts();

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WAKE_BUTTON,INPUT);
  digitalWrite(WAKE_BUTTON, HIGH);

/////////////////////////////////
#if UART_ENABLED == 1
  Serial.begin(UART_SPEED);
#endif

////////////////////////////////
#if LCD_AVAILABLE == 1
  #if LCD_TYPE == LCD1602
    lcd.init();                     
    lcd.backlight();// Включаем подсветку дисплея
  #endif
#endif

////////////////////////////////
#if INTERNAL_SENSOR == BME280
    #if UART_ENABLED == 1
      Serial.println(TEXT_BME280_SELFTEST);
    #endif
    
    #if LCD_AVAILABLE == 1
      lcd.print(TEXT_BME280_SELFTEST);
    #endif
     
    if (!meteo_sensor.begin(0x76)) 
    {                                // Инициализация датчика BME280
      #if UART_ENABLED == 1
       Serial.println(TEXT_BME280_NOT_FOUND);  // Печать сообщения об ошибки
      #endif

      #if LCD_AVAILABLE == 1
        lcd.print(TEXT_BME280_NOT_FOUND);
      #endif
      
       while (1);
    }

    meteo_sensor.setSampling(Adafruit_BME280::MODE_FORCED, /* Режим работы */
    Adafruit_BME280::SAMPLING_X2, /* Температура передискретизация */
    Adafruit_BME280::SAMPLING_X16, /* Давление передискретизация */
    Adafruit_BME280::SAMPLING_X1,  /* humidity*/
    Adafruit_BME280::FILTER_X16); /* Фильрация. */
    
#endif

/////////////////////////////////
  ///initial run
#if INTERNAL_SENSOR == BME280  
   for (unsigned char i = 0; i < (_SERIAL_MEASURE_QUANTITY); i++ )
   {
    meteo_sensor.takeForcedMeasurement();
    delay(INTERNAL_SENSOR_FORCED_DELAY);
    _temp_int.set_new_value(meteo_sensor.readTemperature());
    _press.set_new_value(meteo_sensor.readPressure()/100.0F);
    _hum.set_new_value(meteo_sensor.readHumidity());
   }
     /// Working
   TaskManager::SetTask_(read_meteo,0);
#endif
}

void loop()
{
  //Serial.println("Queue is processing");
  TaskManager::ProcessTaskQueue_();
}
