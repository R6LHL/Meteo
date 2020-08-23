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
  
  extern void compare_Temp(void);
  extern void compare_Press(void);
  extern void compare_Hum(void);
  
  extern void device_sleep_(void);
  extern void device_wake(void);
  
/////////////////////////TASKS DEFINITIONS////////////////////////
  void read_meteo(void)
  {
    
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
    TaskManager::SetTask_(collect_meteo, 750);
  }
  
 void collect_meteo(void)
  {   
    
#if INTERNAL_SENSOR == BME280  
    _temp_int.set_new_value(meteo_sensor.readTemperature());
    _press.set_new_value(meteo_sensor.readPressure()/100.0F);
    _hum.set_new_value(meteo_sensor.readHumidity());
#endif

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

      external_temperature = tempInt + (tempFloat / 16);
      _temp_ext.set_new_value(external_temperature);
  
#endif

    
    
    TaskManager::SetTask_(wake_or_sleep,0);
    TaskManager::SetTask_(read_meteo, _SENSOR_ASK_DELAY_MS);
  }

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
    Serial.print(TEXT_TEMPERATURE_IS);
    Serial.print(_temp_ext.get_middle_value());
    //Serial.print(external_temperature);         //DEBUG
    Serial.println(TEXT_CELSIUS_DEGREE);
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_TEMPERATURE_IS);
    lcd.setCursor(0,1);
    lcd.print(_temp_ext.get_middle_value());
    //lcd.print(external_temperature);            //DEBUG
    lcd.print(TEXT_CELSIUS_DEGREE);
#endif
            
    TaskManager::SetTask_(compare_Temp,_SCREEN_DELAY);
  }

  void compare_Temp(void)
  {
#if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_TEMPERATURE_IS);
    lcd.setCursor(0,1);
#endif

#if UART_ENABLED == 1  
    Serial.print(TEXT_TEMPERATURE_IS);
#endif

    if (_temp_int.get_value_state() == Serial_measure::Value_state::falling)
    {
      #if LCD_TYPE == LCD1602  
        lcd.print(TEXT_FALLING);
      #endif
      
      #if UART_ENABLED == 1  
        Serial.print(TEXT_FALLING);
      #endif
    }

    else if (_temp_int.get_value_state() == Serial_measure::Value_state::rising)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(TEXT_RISING);
      #endif

      #if UART_ENABLED == 1 
        Serial.print(TEXT_RISING);
      #endif
    }

    else if (_temp_int.get_value_state() == Serial_measure::Value_state::stable)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(TEXT_STABLE);
      #endif
      
      #if UART_ENABLED == 1 
        Serial.print(TEXT_STABLE);
      #endif
    }

    else {TaskManager::SetTask_(print_meteo_Press,0); return;}

    #if LCD_TYPE == LCD1602 
      lcd.print(_temp_int.get_state_not_change_times());
      lcd.print(TEXT_MIN);
    #endif
      
    #if UART_ENABLED == 1 
      Serial.print(_temp_int.get_state_not_change_times());
      Serial.println(TEXT_MIN);
    #endif
    
    TaskManager::SetTask_(print_meteo_Press,_SCREEN_DELAY);
  }
//////PRINT Pressure/////////
  void  print_meteo_Press(void)
  {
    #if UART_ENABLED == 1 
      Serial.print(TEXT_PRESSURE_IS);
      Serial.print(_press.get_middle_value());
      Serial.println(TEXT_HPA);
    #endif

    #if LCD_TYPE == LCD1602 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(TEXT_PRESSURE_IS);
      lcd.setCursor(0,1);
      lcd.print(_press.get_middle_value());
      lcd.print(TEXT_HPA);
    #endif
    
    TaskManager::SetTask_(compare_Press,_SCREEN_DELAY);
  }

  void compare_Press(void)
  {
    #if LCD_TYPE == LCD1602 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(TEXT_PRESSURE_IS);
      lcd.setCursor(0,1);
    #endif

    #if UART_ENABLED == 1 
      Serial.print(TEXT_PRESSURE_IS);
    #endif

    if (_press.get_value_state() == Serial_measure::Value_state::falling)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(TEXT_FALLING);
      #endif

      #if UART_ENABLED == 1
        Serial.print(TEXT_FALLING);
      #endif
    }

    else if (_press.get_value_state() == Serial_measure::Value_state::rising)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(TEXT_RISING);
      #endif

      #if UART_ENABLED == 1
        Serial.print(TEXT_RISING);
      #endif
    }

    else if (_press.get_value_state() == Serial_measure::Value_state::stable)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(TEXT_STABLE);
      #endif
      
      #if LCD_TYPE == LCD1602
        Serial.print(TEXT_STABLE); 
      #endif   
    }
    
    else {TaskManager::SetTask_(print_meteo_Hum,0); return;}

    #if LCD_TYPE == LCD1602
      lcd.print(_press.get_state_not_change_times());
      lcd.print(TEXT_MIN);
    #endif

    #if UART_ENABLED == 1
      Serial.print(_press.get_state_not_change_times());
      Serial.println(TEXT_MIN);
    #endif
    
    TaskManager::SetTask_(print_meteo_Hum,_SCREEN_DELAY);
  }
  
//////PRINT humidity/////////
  void  print_meteo_Hum(void)
  {
   #if UART_ENABLED == 1
    Serial.print(TEXT_HUMIDITY_IS);
    Serial.print(_hum.get_middle_value());
    Serial.println(TEXT_PERCENT_SIGN);
   #endif
   
   #if LCD_TYPE == LCD1602
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print(TEXT_HUMIDITY_IS);
     lcd.setCursor(0,1);
     lcd.print(_hum.get_middle_value());
     lcd.print(TEXT_PERCENT_SIGN);
   #endif 
       
   TaskManager::SetTask_(compare_Hum,_SCREEN_DELAY);
  }

  void compare_Hum(void)
  {
    #if LCD_TYPE == LCD1602
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(TEXT_HUMIDITY_IS);
      lcd.setCursor(0,1);
    #endif

    #if UART_ENABLED == 1
      Serial.print(TEXT_HUMIDITY_IS);
    #endif

    if (_hum.get_value_state() == Serial_measure::Value_state::falling)
    {
      #if LCD_TYPE == LCD1602
        lcd.print(TEXT_FALLING);
      #endif

      #if UART_ENABLED == 1
        Serial.print(TEXT_FALLING);
      #endif
    }

    else if (_hum.get_value_state() == Serial_measure::Value_state::rising)
    {
      #if LCD_TYPE == LCD1602
        lcd.print(TEXT_RISING);
      #endif

      #if UART_ENABLED == 1
        Serial.print(TEXT_RISING);
      #endif
    }

    else if (_hum.get_value_state() == Serial_measure::Value_state::stable)
    {
      #if LCD_TYPE == LCD1602
        lcd.print(TEXT_STABLE);
      #endif

      #if UART_ENABLED == 1
        Serial.print(TEXT_STABLE);   
      #endif
    }
    
    else {TaskManager::SetTask_(device_sleep_,0); return;}
    
    #if LCD_TYPE == LCD1602
      lcd.print(_hum.get_state_not_change_times());
      lcd.print(TEXT_MIN);
    #endif
      
    #if UART_ENABLED == 1
      Serial.print(_hum.get_state_not_change_times());
      Serial.println(TEXT_MIN);
    #endif
    
    TaskManager::SetTask_(print_room_Temp,_SCREEN_DELAY);
  }

    void print_room_Temp(void)
  {
#if UART_ENABLED == 1    
    Serial.print(TEXT_ROOM_TEMPERATURE);
    Serial.print(_temp_int.get_middle_value());
    Serial.println(TEXT_CELSIUS_DEGREE);
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(TEXT_ROOM_TEMPERATURE);
    lcd.setCursor(0,1);
    lcd.print(_temp_int.get_middle_value());
    lcd.print(TEXT_CELSIUS_DEGREE);
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

 //////////////////////////////////////////////////////



#endif
