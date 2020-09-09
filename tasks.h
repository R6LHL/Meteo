#ifndef _TASKS_H
#define _TASKS_H

#include "headers.h"

//Tasks declaration
 
  extern void read_meteo(void);
  extern void collect_meteo(void);
  extern void wake_or_sleep(void);
  
  extern void print_meteo_Temp(void);
  extern void print_meteo_Press(void);
  extern void print_meteo_Hum(void);
  extern void print_room_Temp(void);
  extern void print_date_time(void);//!!!!!!!!!

  extern void led_power_high(void);
  extern void led_power_low(void);
  extern void batt_control(void);
  
  extern void collect_ext_temp(void);
  extern void collect_int_temp(void);
  extern void collect_pressure(void);
  extern void collect_humidity(void);

  extern void compare_Temp(void);
  extern void compare_Temp1(void);
  
  extern void compare_pressure(void);
  extern void compare_pressure1(void);
  
  extern void compare_humidity(void);
  extern void compare_humidity1(void);
  
  extern void compare_room_temp(void);
  extern void compare_room_temp1(void);
  
  extern void print_supply_voltage(void);
   
  extern void device_sleep_(void);
  extern void device_wake(void);
  extern void check_UART(void); //!!!!!!!!!!!!!!
  
/////////////////////////TASKS DEFINITIONS////////////////////////
  void read_meteo(void)
  {
    //DateTime now = Clock(now);
#if EXTERNAL_SENSOR == DS18B20
    noInterrupts();
    ext_temp_sens.reset();
    ext_temp_sens.select(ext_temp_addr);
    ext_temp_sens.write(0x44,1); // start conversion
    interrupts();
#endif

#if INTERNAL_SENSOR == BME280
    meteo_sensor.takeForcedMeasurement();
#endif  

    TaskManager::SetTask_(collect_ext_temp, 750);
  }
  
 void collect_ext_temp(void)
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
      volatile unsigned char iteration = (now.minute()%10);
      ext_temp0_10.mov_measure(external_temperature, iteration);
      iteration = now.minute()/10;
      ext_temp10_60.mov_measure(ext_temp0_10.get_mid_value(), iteration);
      iteration = now.hour();
      ext_temp60_1440.mov_measure(ext_temp10_60.get_mid_value(), iteration);
      iteration = now.day();
      ext_temp1440_10080.mov_measure(ext_temp60_1440.get_mid_value(), iteration);
#endif

#if INTERNAL_SENSOR == BME280
    TaskManager::SetTask_(collect_int_temp,0);
#endif    

    TaskManager::SetTask_(read_meteo, _SENSOR_ASK_DELAY_MS);
  }

#if INTERNAL_SENSOR == BME280
void collect_int_temp(void)
{
   ////STATISTIC
   DateTime now = Clock.now();
      volatile unsigned char iteration = (now.minute()%10);
      inner_temp0_10.mov_measure(meteo_sensor.readTemperature(), iteration);
      iteration = now.minute()/10;
      inner_temp10_60.mov_measure(inner_temp0_10.get_mid_value(), iteration);
      iteration = now.hour();
      inner_temp60_1440.mov_measure(inner_temp10_60.get_mid_value(), iteration);
      iteration = now.day();
      inner_temp1440_10080.mov_measure(inner_temp60_1440.get_mid_value(), iteration);
     
  
  TaskManager::SetTask_(collect_pressure,0);
}

void collect_pressure(void)
{
     ////STATISTIC
    DateTime now = Clock.now();
      volatile unsigned char iteration = (now.minute()%10);
      pressure0_10.mov_measure((meteo_sensor.readPressure()/100.0F), iteration);
      iteration = now.minute()/10;
      pressure10_60.mov_measure(pressure0_10.get_mid_value(), iteration);
      iteration = now.hour(); 
      pressure60_1440.mov_measure(pressure10_60.get_mid_value(), iteration);
      iteration = now.day();
      pressure1440_10080.mov_measure(pressure60_1440.get_mid_value(), iteration);
      
  
  TaskManager::SetTask_(collect_humidity,0);
}

void collect_humidity(void)
{
  ////STATISTIC
     DateTime now = Clock.now();
      volatile unsigned char iteration = (now.minute()%10);
      humidity0_10.mov_measure(meteo_sensor.readHumidity(), iteration);
      iteration = now.minute()/10;
      humidity10_60.mov_measure(humidity0_10.get_mid_value(), iteration);   
      iteration = now.hour();;
      humidity60_1440.mov_measure(humidity10_60.get_mid_value(), iteration);
      iteration = now.day();
      humidity1440_10080.mov_measure(humidity60_1440.get_mid_value(), iteration);
      
  TaskManager::SetTask_(wake_or_sleep,0);
}
#endif

  void wake_or_sleep(void)
  {
    if (device_sleep == false)
    {
    TaskManager::SetTask_(print_meteo_Temp,0);
    }
    else { (TaskManager::SetTask_(device_sleep_,0));}
    TaskManager::DeleteTask_(read_meteo);
  }
  
////PRINTING RESULTS////////////////////////////
//////PRINT Temperature/////////
  void print_meteo_Temp(void)
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
            
    TaskManager::SetTask_(compare_Temp,_SCREEN_DELAY);
  }

  void compare_Temp(void)
  {
#if LCD_TYPE == LCD1602 

   //
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_D10M);
    lcd.print(ext_temp0_10.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_D1H);
    lcd.print(ext_temp10_60.get_delta());
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_D10M);
    Serial.println(ext_temp0_10.get_delta());
    Serial.print(TEXT_D1H);
    Serial.println(ext_temp10_60.get_delta());
#endif
    
    TaskManager::SetTask_(compare_Temp1,_SCREEN_DELAY);
  }

void compare_Temp1(void)
{
  #if LCD_TYPE == LCD1602
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(TEXT_D24H);
  lcd.print(ext_temp60_1440.get_delta());
  lcd.setCursor(0,1);
  lcd.print(TEXT_D7D);
  lcd.print(ext_temp1440_10080.get_delta());
  #endif

  #if UART_ENABLED == 1
  Serial.print(TEXT_D24H);
  Serial.println(ext_temp60_1440.get_delta());
  Serial.print(TEXT_D7D);
  Serial.println(ext_temp1440_10080.get_delta());
  #endif

  TaskManager::SetTask_(print_meteo_Press,_SCREEN_DELAY);
}
 
//////PRINT Pressure/////////
  void  print_meteo_Press(void)
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
    
    TaskManager::SetTask_(compare_pressure,_SCREEN_DELAY);
  }
void compare_pressure(void)  
{
  #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_D10M);
    lcd.print(pressure0_10.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_D1H);
    lcd.print(pressure10_60.get_delta());
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_D10M);
    Serial.println(pressure0_10.get_delta());
    Serial.print(TEXT_D1H);
    Serial.println(pressure10_60.get_delta());
#endif

    TaskManager::SetTask_(compare_pressure1,_SCREEN_DELAY);
}

void compare_pressure1(void)  
{
  #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_D24H);
    lcd.print(pressure60_1440.get_delta());
    lcd.setCursor(0,1);
    lcd.print(TEXT_D7D);
    lcd.print(pressure1440_10080.get_delta());
  #endif

  #if UART_ENABLED == 1  
    Serial.print(TEXT_D24H);
    Serial.println(pressure60_1440.get_delta());
    Serial.print(TEXT_D7D);
    Serial.println(pressure1440_10080.get_delta());
  #endif

  TaskManager::SetTask_(print_meteo_Hum,_SCREEN_DELAY);
}

//////PRINT humidity/////////
void  print_meteo_Hum(void)
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
    
    TaskManager::SetTask_(compare_humidity,_SCREEN_DELAY);
  }

void compare_humidity(void)  
{
    #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_D10M);
    lcd.print(humidity0_10.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_D1H);
    lcd.print(humidity10_60.get_delta());
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_D10M);
    Serial.println(humidity0_10.get_delta());
    Serial.print(TEXT_D1H);
    Serial.println(humidity10_60.get_delta());
#endif
    
    TaskManager::SetTask_(compare_humidity1,_SCREEN_DELAY);
}

void compare_humidity1(void)
{
  #if LCD_TYPE == LCD1602    
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(TEXT_D24H);
  lcd.print(humidity60_1440.get_delta());
  lcd.setCursor(0,1);
  lcd.print(TEXT_D7D);
  lcd.print(humidity1440_10080.get_delta());
  #endif

  #if UART_ENABLED == 1  
    Serial.print(TEXT_D24H);
    Serial.println(humidity60_1440.get_delta());
    Serial.print(TEXT_D7D);
    Serial.println(humidity1440_10080.get_delta());
  #endif

  TaskManager::SetTask_(print_room_Temp,_SCREEN_DELAY);
}

void print_room_Temp(void)
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
    
    TaskManager::SetTask_(compare_room_temp,_SCREEN_DELAY);
  }

void compare_room_temp(void)
{
  #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_D10M);
    lcd.print(inner_temp0_10.get_delta());
    lcd.print(SPACE_DEVIDER);
    lcd.setCursor(0,1);
    lcd.print(TEXT_D1H);
    lcd.print(inner_temp10_60.get_delta());
    
    
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_D10M);
    Serial.println(inner_temp0_10.get_delta());
    Serial.print(TEXT_D1H);
    Serial.println(inner_temp10_60.get_delta());
#endif
            
    TaskManager::SetTask_(compare_room_temp1,_SCREEN_DELAY);
}

void compare_room_temp1(void)
{
  #if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_D24H);
    lcd.print(inner_temp60_1440.get_delta());
    lcd.setCursor(0,1);
    lcd.print(TEXT_D7D);
    lcd.print(inner_temp1440_10080.get_delta());
    
  #endif

  #if UART_ENABLED == 1  
    Serial.print(TEXT_D24H);
    Serial.println(inner_temp60_1440.get_delta());
    Serial.print(TEXT_D7D);
    Serial.println(inner_temp1440_10080.get_delta());
  #endif

  TaskManager::SetTask_(print_date_time,_SCREEN_DELAY);
}

#if REAL_TIME_CLOCK == DS3231_

  void print_date_time (void)
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

    TaskManager::SetTask_(print_supply_voltage,_SCREEN_DELAY);
  }

#endif

void print_supply_voltage(void)
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

   TaskManager::SetTask_(device_sleep_,_SCREEN_DELAY); 
}

  void device_sleep_(void)
  {
    #if LCD_TYPE == LCD1602
      lcd.clear();
      lcd.noBacklight();
    #endif
    
    device_sleep = true;

     TaskManager::SetTask_(read_meteo, _SENSOR_ASK_DELAY_MS);
     TaskManager::DeleteTask_(print_meteo_Temp);

    noInterrupts();
    PCICR |= (1<<PCIE0); //Wait for button press to wake up device
    EIMSK |= (1<<INT0);
    interrupts();
  }

 void device_wake(void)
 {
  #if LCD_TYPE == LCD1602
    lcd.backlight();
  #endif
  
  device_sleep = false;
  
  TaskManager::DeleteTask_(read_meteo);
  TaskManager::SetTask_(print_meteo_Temp,0);
 }

 void checkUART(void)
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
  TaskManager::SetTask_(checkUART,100);
 }

 void led_power_high(void)
 {
  digitalWrite(LED_BUILTIN, HIGH);
  TaskManager::SetTask_(led_power_low,500);
 }

 void led_power_low(void)
 {
  digitalWrite(LED_BUILTIN, LOW);
  TaskManager::SetTask_(led_power_high,500);
 }

 void batt_control(void)
 {
    int battery = analogRead(SUPPLY_VOLTAGE_ANALOG_PIN);
    float voltage = (battery) * (AREF_VOLTAGE) * (BATT_VOLTAGE_DIVIDER);
    voltage = voltage / 1024;
    voltage += CALIBRATION_ADDITIVE;
    /* DEBUG
    Serial.println(battery);
    Serial.println(AREF_VOLTAGE);
    Serial.println(voltage);
    END DEBUG*/
    if(voltage <= 3.8){TaskManager::SetTask_(led_power_high, 0);}
    else if((voltage > 3.8) && (voltage <= 4.1))
    {
      TaskManager::DeleteTask_(led_power_high);
      TaskManager::DeleteTask_(led_power_low);
    }
    if (voltage > 4.1){digitalWrite(LED_BUILTIN, HIGH);}
    else if ((voltage <= 4.1) && (voltage > 3.8)){digitalWrite(LED_BUILTIN, LOW);}

    TaskManager::SetTask_(batt_control, 1000);
 }
 //////////////////////////////////////////////////////



#endif
