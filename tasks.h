#ifndef _TASKS_H
#define _TASKS_H

#include "headers.h"

  extern void SYS_wake_or_sleep(void);
  extern void SYS_led_power_high(void);
  extern void SYS_led_power_low(void);
  extern void SYS_batt_control(void);
  
  extern void SYS_device_sleep(void);
  extern void SYS_device_wake(void);
  extern void SYS_check_UART(void);

  extern void BACKGND_read_meteo(void);
  extern void BACKGND_collect_ext_temp(void);
  extern void BACKGND_collect_int_temp(void);
  extern void BACKGND_collect_pressure(void);
  extern void BACKGND_collect_humidity(void);
  
  extern void UI_print_meteo_Temp(void);
  extern void UI_print_meteo_Press(void);
  extern void UI_print_meteo_Hum(void);
  extern void UI_print_room_Temp(void);
  extern void UI_print_date_time(void);//!!!!!!!!!
  
  extern void UI_compare_Temp(void);
  extern void UI_compare_Temp1(void);
  extern void UI_compare_Temp2(void);
  extern void UI_compare_Temp3(void);
  
  extern void UI_compare_pressure(void);
  extern void UI_compare_pressure1(void);
  extern void UI_compare_pressure2(void);
  extern void UI_compare_pressure3(void);
  
  extern void UI_compare_humidity(void);
  extern void UI_compare_humidity1(void);
  extern void UI_compare_humidity2(void);
  extern void UI_compare_humidity3(void);
  
  extern void UI_compare_room_temp(void);
  extern void UI_compare_room_temp1(void);
  
  extern void UI_print_supply_voltage(void);

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void SYS_device_sleep(void)
{
    #if LCD_TYPE == LCD1602
      lcd.clear();
      lcd.noBacklight();
    #endif
    
    device_sleep = true;

     TaskManager::SetTask_(BACKGND_read_meteo, _SENSOR_ASK_DELAY_MS);
     TaskManager::DeleteTask_(UI_print_meteo_Temp);

    noInterrupts();
    PCICR |= (1<<PCIE0); //Wait for button press to wake up device
    EIMSK |= (1<<INT0);
    interrupts();
}
///////////////////////////////////////////////////////////////////////

void SYS_device_wake(void)
  {
    #if LCD_TYPE == LCD1602
      lcd.backlight();
    #endif
  
    device_sleep = false;
  
    TaskManager::DeleteTask_(BACKGND_read_meteo);
    TaskManager::SetTask_(UI_print_meteo_Temp,0);
}
//////////////////////////////////////////////////////

void SYS_checkUART(void)
{
    if (Serial.available())
    {
    bool gotString = false;
    char inChar;
    char letter[5];
    unsigned char first_digit;
    unsigned char second_digit;
    unsigned char i = 0;
    //Read data from Serial buffer
      while (gotString == false)
      {
        inChar = Serial.read();
        letter[i] = inChar;
        i++;
        if (inChar == '+') 
        {
          gotString = true;
       }
      }
  
    
//Serial.println(letter);
    
  //set hours
   if (letter[0] == 'h')
   {
      first_digit = (byte)((letter[1] - 48) * 10);
      second_digit =(byte)(letter[2] - 48);
      rtc.setHour(first_digit + second_digit);
   }
  //set minutes
  if (letter[0] == 'm')
  {
    first_digit = (byte)((letter[1] - 48) * 10);
    second_digit =(byte)(letter[2] - 48);
    rtc.setMinute(first_digit + second_digit);
   }
  //set seconds
   if (letter[0] == 's')
   {
      first_digit = (byte)((letter[1] - 48) * 10);
      second_digit =(byte)(letter[2] - 48);
      rtc.setSecond(first_digit + second_digit);
   }

  //setDate
   if (letter[0] == 'd')
   {
    first_digit = (byte)((letter[1] - 48) * 10);
    second_digit =(byte)(letter[2] - 48);
    rtc.setDate(first_digit + second_digit);
   }
  //set month
   if (letter[0] == 'm' && letter[1] == 'o')
   {  
     first_digit = (byte)((letter[2] - 48) * 10);
     second_digit =(byte)(letter[3] - 48);
      rtc.setMonth(first_digit + second_digit);
   }
  //set year
    if (letter[0] == 'y')
   {
    first_digit = (byte)((letter[1] - 48) * 10);
    second_digit =(byte)(letter[2] - 48);
    rtc.setYear(first_digit + second_digit);
   }
  }
  TaskManager::SetTask_(SYS_checkUART,100);
}
/////////////////////////////////////////////////

void SYS_led_power_high(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  TaskManager::SetTask_(SYS_led_power_low,500);
}
////////////////////////////////////////////////////////////////////////////////

void SYS_led_power_low(void)
{
  digitalWrite(LED_BUILTIN, LOW);
  TaskManager::SetTask_(SYS_led_power_high,500);
}
/////////////////////////////////////////////////////////////////////////////////

void SYS_batt_control(void)
{
    int battery = analogRead(SUPPLY_VOLTAGE_ANALOG_PIN);
    float voltage = (battery) * (AREF_VOLTAGE) * (BATT_VOLTAGE_DIVIDER);
    voltage = voltage / 1024;
    voltage += CALIBRATION_ADDITIVE;
    
    if(voltage <= BATTERY_LOW){TaskManager::SetTask_(SYS_led_power_high, 0);}
    else if((voltage > BATTERY_LOW) && (voltage <= BATTERY_HIGH))
    {
      TaskManager::DeleteTask_(SYS_led_power_high);
      TaskManager::DeleteTask_(SYS_led_power_low);
    }
    
    /* HIGH VOLTAGE CONTROL WITH BUILTIN LED
    if (voltage > BATTERY_HIGH){digitalWrite(LED_BUILTIN, HIGH);}
    else if ((voltage <= BATTERY_HIGH) && (voltage > BATTERY_LOW)){digitalWrite(LED_BUILTIN, LOW);}
    */

    TaskManager::SetTask_(SYS_batt_control, BATT_CONTROL_PERIOD_MS);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

void SYS_wake_or_sleep(void)
{
   if (device_sleep == false)
   {
     TaskManager::SetTask_(UI_print_meteo_Temp,0);
   }
    else { (TaskManager::SetTask_(SYS_device_sleep,0));}
    TaskManager::DeleteTask_(BACKGND_read_meteo);
}

//////////////////////////////////////////////////////////
//
//BACKGROUND
//
///////////////////////////////////////////////////////////

void BACKGND_read_meteo(void)
{
    DateTime now = Clock.now();
#if EXTERNAL_SENSOR == DS18B20
    noInterrupts();
    ext_temp_sens.reset();
    ext_temp_sens.select(ext_temp_addr);
    ext_temp_sens.write(0x44, 1); // start conversion
    interrupts();
#endif

#if INTERNAL_SENSOR == BME280
 #define BME280_DELAY (750)
    meteo_sensor.takeForcedMeasurement();
#endif  

    TaskManager::SetTask_(BACKGND_collect_ext_temp, BME280_DELAY);
}
////////////////////////////////////////////////////////////////////  

void BACKGND_collect_ext_temp(void)
{   
#if EXTERNAL_SENSOR == DS18B20
    noInterrupts();
    ext_temp_sens.reset();
    ext_temp_sens.select(ext_temp_addr);
    ext_temp_sens.write(0xBE,0); // make sensor transmit scratchpad
    for(char i = 0; i < BYTE_ITERATOR; i++)
    {
    ext_temp_data[i] = ext_temp_sens.read();
    }
    interrupts();
    HighByte = ext_temp_data[1];
    LowByte = ext_temp_data[0];

    unsigned char temp;

    if ((HighByte & 0b11110000) != 0)
    {
    temp = ((unsigned int)HighByte << 8) | LowByte;
    temp = ~temp + 1;
    LowByte = temp;
    HighByte = temp >> 8;
    } 

    tempInt = ((HighByte & 7)<<4)|(LowByte >> 4);
    tempFloat = (LowByte & 15);

    external_temperature = (tempInt + (tempFloat / 16));
      
 ////STATISTIC
    DateTime now = Clock.now();
    unsigned char iteration = (now.minute()%10);
    //DEBUG
    Serial.print(F("now.minute() "));
    Serial.println(now.minute(), DEC);
    Serial.print(F("now.minute()%10 "));
    Serial.println(now.minute()%10,DEC);
    //END DEBUG
    
    ext_temp0_10.mov_measure(external_temperature, iteration);
        
    iteration = now.minute()/10;
    //DEBUG
    Serial.print(F("now.minute()/10 "));
    Serial.println(now.minute()/10, DEC);
    //END DEBUG
    iteration = now.minute()/10;
    ext_temp10_60.mov_measure(ext_temp0_10.get_mid_value(), iteration);
    Serial.print(F("ext_temp0_10.get_mid_value() "));
    Serial.println(ext_temp0_10.get_mid_value());
    
    iteration = now.hour();
    Serial.print(F("now.hour() "));
    Serial.println(now.hour());

    ext_temp60_1440.mov_measure(ext_temp10_60.get_mid_value(), iteration);
    Serial.print(F("ext_temp10_60.get_mid_value() "));
    Serial.println(ext_temp10_60.get_mid_value());
    //iteration = now.dayOfTheWeek();
    
    
     //DEBUG
    //Serial.print(F("now.dayOfTheWeek() "));
    //Serial.println(now.dayOfTheWeek());
    
    //END DEBUG 
    
    ext_temp1440_10080.mov_measure(ext_temp60_1440.get_mid_value(), iteration);
    //DEBUG
    
    Serial.print(F("ext_temp60_1440.get_mid_value() "));
    Serial.println(ext_temp60_1440.get_mid_value());
    //END DEBUG
    
    
#endif

#if INTERNAL_SENSOR == BME280
    TaskManager::SetTask_(BACKGND_collect_int_temp,0);
#endif    
    TaskManager::SetTask_(BACKGND_read_meteo, _SENSOR_ASK_DELAY_MS);
}
///////////////////////////////////////////////////////////////////////////////////

void BACKGND_collect_int_temp(void)
{
   ////STATISTIC
    DateTime now = Clock.now();
    unsigned char iteration = (now.minute()%10);
    inner_temp0_10.mov_measure(meteo_sensor.readTemperature(), iteration);
    iteration = now.minute()/10;
    inner_temp10_60.mov_measure(inner_temp0_10.get_mid_value(), iteration);
    iteration = now.hour();
    inner_temp60_1440.mov_measure(inner_temp10_60.get_mid_value(), iteration);
    /*
    iteration = now.day();
    inner_temp1440_10080.mov_measure(inner_temp60_1440.get_mid_value(), iteration);
    */
  
  TaskManager::SetTask_(BACKGND_collect_pressure,0);
}
//////////////////////////////////////////////////////////////////////////////////////

void BACKGND_collect_pressure(void)
{
    ////STATISTIC
    DateTime now = Clock.now();
    unsigned char iteration = (now.minute()%10);
    pressure0_10.mov_measure((meteo_sensor.readPressure()/100.0F), iteration);
    iteration = now.minute()/10;
    pressure10_60.mov_measure(pressure0_10.get_mid_value(), iteration);
    iteration = now.hour(); 
    pressure60_1440.mov_measure(pressure10_60.get_mid_value(), iteration);
    /*
    iteration = now.dayOfTheWeek();
    pressure1440_10080.mov_measure(pressure60_1440.get_mid_value(), iteration); 
  */
  TaskManager::SetTask_(BACKGND_collect_humidity,0);
}
////////////////////////////////////////////////////////////////////////////////////

void BACKGND_collect_humidity(void)
{
  ////STATISTIC
    DateTime now = Clock.now();
    unsigned char iteration = (now.minute()%10);
    humidity0_10.mov_measure(meteo_sensor.readHumidity(), iteration);
    iteration = now.minute()/10;
    humidity10_60.mov_measure(humidity0_10.get_mid_value(), iteration);   
    iteration = now.hour();
    humidity60_1440.mov_measure(humidity10_60.get_mid_value(), iteration);
    /*
    iteration = now.dayOfTheWeek();
    humidity1440_10080.mov_measure(humidity60_1440.get_mid_value(), iteration);
    */  
  TaskManager::SetTask_(SYS_wake_or_sleep,0);
}
////////////////////////////////////////////////////////////////////////////////////

void UI_print_meteo_Temp(void)
{
#if UART_ENABLED == 1  
    DateTime now = Clock.now();
    //strcpy(text_buffer,(char*)pgm_read_word(TEXT_TEMPERATURE_IS));
    Serial.print(TEXT_TEMPERATURE_IS);
    Serial.print(external_temperature);
    Serial.println(TEXT_CELSIUS_DEGREE);
    //debug
    //Serial.println(ext_temp0_10.get_iterator());
    //Serial.println(now.minute()%10);
    //Serial.println(ext_temp10_60.get_iterator());
    //Serial.println(now.minute()/10);
    //Serial.println(ext_temp60_1440.get_iterator());
    //Serial.println(now.hour());
    //end debug
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    //strcpy(text_buffer,(char*)pgm_read_word(TEXT_TEMPERATURE_IS));
    lcd.print(TEXT_TEMPERATURE_IS);
    lcd.setCursor(0,1);
    lcd.print(external_temperature);
    lcd.print(TEXT_CELSIUS_DEGREE);
#endif
            
    TaskManager::SetTask_(UI_compare_Temp,_SCREEN_DELAY);
}
/////////////////////////////////////////////////////////////////////////

void UI_compare_Temp(void)
{
 DateTime now = Clock.now();
 unsigned char iteration = now.hour();

#if LCD_TYPE == LCD1602 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DTe);
    lcd.print(TEXT_1H );
    lcd.print(ext_temp10_60.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_DTe);
    lcd.print(TEXT_3H);
    lcd.print(ext_temp60_1440.get_delta(iteration,3));
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DTe);
    Serial.print(TEXT_1H);
    Serial.println(ext_temp10_60.get_delta());
    Serial.print(TEXT_DTe);
    Serial.print(TEXT_3H);
    Serial.println(ext_temp60_1440.get_delta(iteration,3));
#endif
    
    TaskManager::SetTask_(UI_compare_Temp1,_SCREEN_DELAY);
} 
//////////////////////////////////////////////////////////////  

void UI_compare_Temp1(void)
{
  DateTime now = Clock.now();
  unsigned char iteration = now.hour();
  
#if LCD_TYPE == LCD1602
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(TEXT_DTe);    
  lcd.print(TEXT_6H);
  lcd.print(ext_temp60_1440.get_delta(iteration,6));
  lcd.setCursor(0,1);
  lcd.print(TEXT_DTe);
  lcd.print(TEXT_12H);
  lcd.print(ext_temp60_1440.get_delta(iteration,12));
#endif

#if UART_ENABLED == 1
  Serial.print(TEXT_DTe);
  Serial.print(TEXT_6H);
  Serial.println(ext_temp60_1440.get_delta(iteration,6));
  Serial.print(TEXT_DTe);
  Serial.print(TEXT_12H);
  Serial.println(ext_temp60_1440.get_delta(iteration,12));
#endif

  TaskManager::SetTask_(UI_compare_Temp2,_SCREEN_DELAY);
}
/////////////////////////////////////////////////////////////

void UI_compare_Temp2(void)
{
  DateTime now = Clock.now();
  
#if LCD_TYPE == LCD1602
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(TEXT_DTe);    
  lcd.print(TEXT_24H);
  lcd.print(ext_temp60_1440.get_delta(now.hour(),24));

  lcd.setCursor(0,1);
  lcd.print(TEXT_DTe);
  lcd.print(TEXT_7D);
  lcd.print(ext_temp1440_10080.get_delta(now.day(),7));
#endif
  
#if UART_ENABLED == 1
  Serial.print(TEXT_DTe);
  Serial.print(TEXT_24H);
  Serial.println(ext_temp60_1440.get_delta(now.hour(),24));
  Serial.print(TEXT_DTe);
  Serial.print(TEXT_7D);
  Serial.println(ext_temp1440_10080.get_delta(now.day(),7));
#endif

  TaskManager::SetTask_(UI_print_meteo_Press,_SCREEN_DELAY);
}
////////////////////////////////////////////////////////////

void UI_print_meteo_Press(void)
{
#if UART_ENABLED == 1    
    Serial.print(TEXT_PRESSURE_IS);
    Serial.print((meteo_sensor.readPressure()/100.0F));
    //Serial.print(external_temperature);         //DEBUG
    Serial.println(TEXT_HPA );
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_PRESSURE_IS);
    lcd.setCursor(0,1);
    lcd.print((meteo_sensor.readPressure()/100.0F));
    //lcd.print(external_temperature);            //DEBUG
    lcd.print(TEXT_HPA );
#endif
    
    TaskManager::SetTask_(UI_compare_pressure,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////  
  
void UI_compare_pressure(void)  
{
  DateTime now = Clock.now();
  unsigned char iteration = now.hour();
  
  #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DP);
    lcd.print(TEXT_1H);
    lcd.print(pressure10_60.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_DP);
    lcd.print(TEXT_3H);
    lcd.print(pressure60_1440.get_delta(iteration,3));
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DP);
    Serial.print(TEXT_1H);
    Serial.println(pressure10_60.get_delta());
    Serial.print(TEXT_DP);
    Serial.print(TEXT_3H);
    Serial.println(pressure60_1440.get_delta(iteration,3));
#endif

    TaskManager::SetTask_(UI_compare_pressure1,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////

void UI_compare_pressure1(void)  
{
  DateTime now = Clock.now();
  unsigned char iteration = now.hour();
  
#if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DP);
    lcd.print(TEXT_6H);
    lcd.print(pressure60_1440.get_delta(iteration,6));
    lcd.setCursor(0,1);
    lcd.print(TEXT_DP);
    lcd.print(TEXT_12H);
    lcd.print(pressure60_1440.get_delta(iteration,12));
#endif

#if UART_ENABLED == 1
    Serial.print(TEXT_DP);
    Serial.print(TEXT_12H);
    Serial.println(pressure60_1440.get_delta(iteration,6));
    Serial.print(TEXT_DP);
    Serial.print(TEXT_12H);
    Serial.println(pressure60_1440.get_delta(iteration,12));
#endif

  TaskManager::SetTask_(UI_compare_pressure2,_SCREEN_DELAY);
}
/////////////////////////////////////////////////////////////////

void UI_compare_pressure2(void)  
{
  DateTime now = Clock.now();
#if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DP);
    lcd.print(TEXT_24H);
    lcd.print(pressure60_1440.get_delta(now.hour(),24));
    lcd.setCursor(0,1);
    lcd.print(TEXT_DP);
    lcd.print(TEXT_7D);
    lcd.print(pressure1440_10080.get_delta(now.day(),7));
#endif

#if UART_ENABLED == 1
    Serial.print(TEXT_DP);
    Serial.print(TEXT_24H);
    Serial.println(pressure60_1440.get_delta(now.hour(),24));
    Serial.print(TEXT_DP);
    Serial.print(TEXT_7D);
    Serial.println(pressure1440_10080.get_delta(now.day(),7));
#endif

  TaskManager::SetTask_(UI_print_meteo_Hum,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////

void UI_print_meteo_Hum(void)
{
#if UART_ENABLED == 1    
  Serial.print(TEXT_HUMIDITY_IS);
    Serial.print(meteo_sensor.readHumidity());
    //Serial.print(external_temperature);         //DEBUG
    Serial.println(TEXT_PERCENT_SIGN );
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_HUMIDITY_IS);
    lcd.setCursor(0,1);
    lcd.print(meteo_sensor.readHumidity());
    //lcd.print(external_temperature);            //DEBUG
    lcd.print(TEXT_PERCENT_SIGN );
#endif
    
    TaskManager::SetTask_(UI_compare_humidity,_SCREEN_DELAY);
}
//////////////////////////////////////////////////////////////

void UI_compare_humidity(void)  
{
    DateTime now = Clock.now();
    unsigned char iteration = now.hour();
    
    #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DH);
    lcd.print(TEXT_1H);
    lcd.print(humidity10_60.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_DH);
    lcd.print(TEXT_3H);
    lcd.print(humidity60_1440.get_delta(iteration,3));
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DH);
    Serial.print(TEXT_1H);
    Serial.println(humidity10_60.get_delta());
    Serial.print(TEXT_DH);
    Serial.print(TEXT_3H);
    Serial.println(humidity60_1440.get_delta(iteration,3));
#endif
    
    TaskManager::SetTask_(UI_compare_humidity1,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////

void UI_compare_humidity1(void)
{
  DateTime now = Clock.now();
  unsigned char iteration = now.hour();
  
#if LCD_TYPE == LCD1602    
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(TEXT_DH);
  lcd.print(TEXT_6H);
  lcd.print(humidity60_1440.get_delta(iteration,6));
  lcd.setCursor(0,1);
  lcd.print(TEXT_DH);
  lcd.print(TEXT_12H);
  lcd.print(humidity60_1440.get_delta(iteration,12));
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DH);
    Serial.print(TEXT_6H);
    Serial.println(humidity60_1440.get_delta(iteration,6));
    Serial.print(TEXT_DH);
    Serial.print(TEXT_12H);
    Serial.println(humidity60_1440.get_delta(iteration,12));
#endif

  TaskManager::SetTask_(UI_compare_humidity2,_SCREEN_DELAY);
}
//////////////////////////////////////////////////////////////

void UI_compare_humidity2(void)
{
  DateTime now = Clock.now();
#if LCD_TYPE == LCD1602    
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(TEXT_DH);
  lcd.print(TEXT_24H);
  lcd.print(humidity60_1440.get_delta(now.hour(),24));
  lcd.setCursor(0,1);
  lcd.print(TEXT_DH);
  lcd.print(TEXT_7D);
  lcd.print(humidity1440_10080.get_delta(now.day(),7));
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DH);
    Serial.print(TEXT_24H);
    Serial.println(humidity60_1440.get_delta(now.hour(),24));
    Serial.print(TEXT_DH);
    Serial.print(TEXT_7D);
    Serial.println(humidity1440_10080.get_delta(now.day(),7));
#endif

  TaskManager::SetTask_(UI_print_room_Temp,_SCREEN_DELAY);
}
////////////////////////////////////////////////////////////////

void UI_print_room_Temp(void)
{
#if UART_ENABLED == 1    
    Serial.print(TEXT_ROOM_TEMPERATURE);
    Serial.print(meteo_sensor.readTemperature());
    //Serial.print(external_temperature);         //DEBUG
    Serial.println(TEXT_CELSIUS_DEGREE  );
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_ROOM_TEMPERATURE);
    lcd.setCursor(0,1);
    lcd.print(meteo_sensor.readTemperature());
    //lcd.print(external_temperature);            //DEBUG
    lcd.print(TEXT_CELSIUS_DEGREE  );
#endif
    
    TaskManager::SetTask_(UI_compare_room_temp,_SCREEN_DELAY);
}
////////////////////////////////////////////////////////////

void UI_compare_room_temp(void)
{
  DateTime now = Clock.now();
  unsigned char iteration = now.hour();
  
#if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DTi);
    lcd.print(TEXT_1H);
    lcd.print(inner_temp10_60.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_DTi);
    lcd.print(TEXT_3H);
    lcd.print(inner_temp60_1440.get_delta(iteration,3));
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DTi);
    Serial.print(TEXT_1H);
    Serial.println(inner_temp10_60.get_delta());
    Serial.print(TEXT_DTi);
    Serial.print(TEXT_3H);
    Serial.println(inner_temp60_1440.get_delta(iteration,3));
#endif
            
    TaskManager::SetTask_(UI_compare_room_temp1,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////////

void UI_compare_room_temp1(void)
{
  DateTime now = Clock.now();
  unsigned char iteration = now.hour();
#if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_DTi);
    lcd.print(TEXT_6H);
    lcd.print(inner_temp60_1440.get_delta(iteration,6));
    
    lcd.setCursor(0,1);
    lcd.print(TEXT_DTi);
    lcd.print(TEXT_12H);
    lcd.print(inner_temp60_1440.get_delta(iteration,12)); 
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_DTi);
    Serial.print(TEXT_6H);
    Serial.println(inner_temp60_1440.get_delta(iteration,6));
    Serial.print(TEXT_DTi);
    Serial.print(TEXT_12H);
    Serial.println(inner_temp60_1440.get_delta(iteration,12));   
#endif

  TaskManager::SetTask_(UI_print_date_time,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////////

void UI_print_date_time (void)
{
  DateTime now = Clock.now();
#if UART_ENABLED == 1    
    if (now.day()<10){Serial.print(TEXT_ZERO);}
    Serial.print(now.day(),DEC);
    Serial.print(DATE_DEVIDER);
    if (now.month()<10){Serial.print(TEXT_ZERO);}
    Serial.print(now.month(),DEC);
    Serial.print(DATE_DEVIDER);
    Serial.println(now.year(),DEC);
      //Serial.print(SPACE_DEVIDER);
      //Serial.println(now.dayOfTheWeek());

    if (now.hour()<10){Serial.print(TEXT_ZERO);}
    Serial.print(now.hour(),DEC);     
    Serial.print(TIME_DEVIDER);
    if (now.minute()<10){Serial.print(TEXT_ZERO);}
    Serial.println(now.minute(),DEC);     
      //Serial.print(TIME_DEVIDER);
      //Serial.println(now.second(),DEC);
#endif

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    if (now.day()<10){lcd.print(TEXT_ZERO);}
    lcd.print(now.day(), DEC);
    lcd.print(DATE_DEVIDER);
    if (now.month()<10){lcd.print(TEXT_ZERO);}
    lcd.print(now.month(),DEC);
    lcd.print(DATE_DEVIDER);
    lcd.print(now.year(),DEC);
      //lcd.print(SPACE_DEVIDER);
      //lcd.print(now.dayOfTheWeek());

    lcd.setCursor(0,1);
    if (now.hour()<10){lcd.print(TEXT_ZERO);}
    lcd.print(now.hour(),DEC);     
    lcd.print(TIME_DEVIDER);
    if (now.minute()<10){lcd.print(TEXT_ZERO);}
    lcd.print(now.minute(),DEC);     
      //lcd.print(TIME_DEVIDER);
      //lcd.print(now.second(),DEC);
#endif

    TaskManager::SetTask_(UI_print_supply_voltage,_SCREEN_DELAY);
}
///////////////////////////////////////////////////////////////////////////////

void UI_print_supply_voltage(void)
{
#if LCD_TYPE == LCD1602
    int battery = analogRead(SUPPLY_VOLTAGE_ANALOG_PIN);
    float voltage = (battery) * (AREF_VOLTAGE) * (BATT_VOLTAGE_DIVIDER);
    voltage = voltage / 1024;
    voltage += CALIBRATION_ADDITIVE;
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_BATT_VOLTAGE);
    lcd.print(voltage);
    lcd.print(TEXT_VOLT_SIGN);
#endif

#if UART_ENABLED == 1
    Serial.print(TEXT_BATT_VOLTAGE);
    //Serial.println (battery); ///debug
    Serial.print(voltage);
    Serial.println(TEXT_VOLT_SIGN);    
#endif

TaskManager::SetTask_(SYS_device_sleep,_SCREEN_DELAY); 
}

#endif //_TASKS_H
