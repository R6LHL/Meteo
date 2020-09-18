
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
  
  Serial_measure_f0_10        inner_temp0_10;
  Serial_measure_f10_60       inner_temp10_60;\
  /*
  Serial_measure_f60_180      inner_temp60_180;
  Serial_measure_f180_360     inner_temp180_360;
  Serial_measure_f360_720     inner_temp360_720;
  */
  Serial_measure_f60_1440    inner_temp60_1440;
  //Serial_measure_f1440_10080  inner_temp1440_10080;
  
  Serial_measure_f0_10        pressure0_10;
  Serial_measure_f10_60       pressure10_60;
  /*
  Serial_measure_f60_180      pressure60_180;
  Serial_measure_f180_360     pressure180_360;
  Serial_measure_f360_720     pressure360_720;
  */
  Serial_measure_f60_1440     pressure60_1440;
  Serial_measure_f1440_10080  pressure1440_10080;

  Serial_measure_f0_10        humidity0_10;
  Serial_measure_f10_60       humidity10_60;
  /*
  Serial_measure_f60_180      humidity60_180;
  Serial_measure_f180_360     humidity180_360;
  Serial_measure_f360_720     humidity360_720;
  */
  Serial_measure_f60_1440     humidity60_1440;
  Serial_measure_f1440_10080  humidity1440_10080;
  
    
#endif

#if LCD_TYPE == LCD1602
  LiquidCrystal_I2C lcd(LCD_INITIAL_CODE,LCD_CHARS,LCD_STRINGS);  // Устанавливаем дисплей
#endif

#ifdef EXTERNAL_SENSOR
  
  #if EXTERNAL_SENSOR == DS18B20
    OneWire ext_temp_sens(ONE_WIRE_PIN);
    byte ext_temp_addr[8];
    byte ext_temp_data[BYTE_ITERATOR];
    byte HighByte;
    byte LowByte; 
    float external_temperature;
    byte tempInt; 
    byte tempFloat;

    Serial_measure_f0_10        ext_temp0_10;
    Serial_measure_f10_60       ext_temp10_60;
    /*
    Serial_measure_f60_180      ext_temp60_180;
    Serial_measure_f180_360     ext_temp180_360;
    Serial_measure_f360_720     ext_temp360_720;
    */
    Serial_measure_f60_1440     ext_temp60_1440;
    Serial_measure_f1440_10080  ext_temp1440_10080;
    
  #endif
#endif

#if REAL_TIME_CLOCK == DS3231_
    RTClib Clock;
    bool century;
    bool am_pm;
    bool tw;
    DS3231 rtc;
    extern DateTime now;
    //DateTime current_date;
#endif

byte system_error_code = 0;
bool device_sleep = false;
bool first_measure = true;
char text_buffer[16];
volatile unsigned char button_pressed;

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

/*
ISR(PCINT0_vect)
{
  PCICR &= ~(1<<PCIE0); //Disble interrupt
  /*
  button_pressed = 0b11100000;
  button_pressed |= (digitalRead(TEMP_EXT_BUTTON)<<0);
  button_pressed |= (digitalRead(TEMP_INT_BUTTON)<<1);
  button_pressed |= (digitalRead(PRESSURE_BUTTON)<<2);
  button_pressed |= (digitalRead(HUMIDITY_BUTTON)<<3);
  button_pressed |= (digitalRead(TIME_BATT_BUTTON)<<4);
  //button_pressed &= ~(KEY_MASK);
  */
  /*
   button_pressed = 0;
  if (digitalRead(TEMP_EXT_BUTTON) == LOW) button_pressed = 8;
  if (digitalRead(TEMP_INT_BUTTON) == LOW) button_pressed = 9;
  if (digitalRead(PRESSURE_BUTTON) == LOW) button_pressed = 10;
  if (digitalRead(HUMIDITY_BUTTON) == LOW) button_pressed = 11;
  if (digitalRead(TIME_BATT_BUTTON) == LOW) button_pressed = 12;

#if DEBUG_MODE == ENABLED
    Serial.print(F("BUTTON_CODE"));
    Serial.println(button_pressed,DEC);
#endif 
  
  TaskManager::SetTask_(SYS_device_wake,500);
}

/*
ISR(WDT_OVF_vect)
{
  //DEBUG
  //Serial.println("Interrupt is working");
  //END DEBUG
  
  TaskManager::TimerTaskService_();
}
*/
//////////////////////////////////////////////////////////////////////////////
////END INTERRUPT HANDLERS
//////////////////////////////////////////////////////////////////////////////

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

//WDTCSR |= (1<<WDIE)|(1<<WDCE)|(1<<WDP2)|(1<<WDP1); //tune watchdog timer for 1ms interrupt (128kHz/128) as TaskManager Timer
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  pinMode(TEMP_EXT_BUTTON,INPUT_PULLUP);
  pinMode(TEMP_INT_BUTTON,INPUT_PULLUP);
  pinMode(PRESSURE_BUTTON,INPUT_PULLUP);
  pinMode(HUMIDITY_BUTTON,INPUT_PULLUP);
  pinMode(TIME_BATT_BUTTON,INPUT_PULLUP);
  
  analogReference(DEFAULT);

/////////////////////////////////
#if UART_ENABLED == 1
  Serial.begin(UART_SPEED);
  UCSR0B |= (1<<RXCIE0);
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
    /*
    #if UART_ENABLED == 1
      Serial.println(TEXT_BME280_SELFTEST);
    #endif
    
    #if LCD_AVAILABLE == 1
      lcd.print(TEXT_BME280_SELFTEST);
    #endif
    */
     
    if (!meteo_sensor.begin(0x76)) 
    {                                // Инициализация датчика BME280
      #if UART_MODE == TRX
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
/*  
#if INTERNAL_SENSOR == BME280  
   for (unsigned char i = 0; i < (_SERIAL_MEASURE_QUANTITY); i++ )
   {
    meteo_sensor.takeForcedMeasurement();
    delay(INTERNAL_SENSOR_FORCED_DELAY);
    _temp_int.set_new_value(meteo_sensor.readTemperature());
    _press.set_new_value(meteo_sensor.readPressure()/100.0F);
    _hum.set_new_value(meteo_sensor.readHumidity());
   }
*/
#if REAL_TIME_CLOCK == DS3231_
  rtc.setClockMode(false); //24h format
#endif

/// Working
   
#if POWER_SUPPLY == AUTONOMOUS   
   TaskManager::SetTask_(SYS_batt_control,0);
#endif
  TaskManager::SetTask_(SYS_checkUART,0);
  TaskManager::SetTask_(BACKGND_read_meteo,0);
  //TaskManager::SetTask_(UI_print_date_time,2000);
  TaskManager::SetTask_(SYS_keyboard_scan,2000);
/*
  PCICR |= (1<<PCIE0); //Wait for button press to wake up device
  PCMSK0 |= (1<<PCINT4)|(1<<PCINT3)|(1<<PCINT2)|(1<<PCINT1)|(1<<PCINT0);
*/

interrupts();

  device_sleep = true;
}

void loop()
{
  //Serial.println("Queue is processing");
  TaskManager::ProcessTaskQueue_();
}
