
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
  Serial_measure_f10_60       inner_temp10_60;
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

#if EXTERNAL_SENSOR == DALLAS_DS18B20
    DS18B20 ext_temp_sens(ONE_WIRE_PIN);
    
    float external_temperature;

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

#include "tasks.h"
#include "interrupts.h"
#include "setup.h"

//////////////////////////////////////////////////////////////////////////////

void setup()
{ 
  Setup_UART();
  Setup_timers();
  Setup_IOpins();
  
  
  Setup_LCD();
  Setup_internal_sensor();
  
#ifdef DEBUG_MODE  
  Serial.print(F("Setup initial tasks...\0"));
#endif
   
#if POWER_SUPPLY == AUTONOMOUS   
   TaskManager::SetTask_(SYS_batt_control,0);
#endif
  TaskManager::SetTask_(SYS_checkUART,0);
  TaskManager::SetTask_(BACKGND_read_meteo,0);
  //TaskManager::SetTask_(UI_print_date_time,2000);
  TaskManager::SetTask_(SYS_keyboard_scan,2000);

  device_sleep = true;

#ifdef DEBUG_MODE  
  Serial.println(F("Done!\0"));
#endif
}

void loop()
{
  //Serial.println("Queue is processing");
  TaskManager::ProcessTaskQueue_();
}
