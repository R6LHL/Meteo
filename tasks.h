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
  
  extern void compare_Temp(void);
  extern void compare_Press(void);
  extern void compare_Hum(void);
  
  extern void device_sleep_(void);
  extern void device_wake(void);
  
/////////////////////////TASKS DEFINITIONS////////////////////////
  void read_meteo(void)
  {
    
#if EXTERNAL_SENSOR == DS18B20
    ext_temp_sens.write(0x44,0); // start conversion with full power supply
#endif

#if INTERNAL_SENSOR == BME280
    meteo_sensor.takeForcedMeasurement();
#endif
   
    TaskManager::SetTask_(collect_meteo, 750);
  }
  
 void collect_meteo(void)
  {   
#if INTERNAL_SENSOR == BME280  
    _temp.set_new_value(meteo_sensor.readTemperature());
    _press.set_new_value(meteo_sensor.readPressure()/100.0F);
    _hum.set_new_value(meteo_sensor.readHumidity());
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
    Serial.print(F("Temperature is "));
    Serial.print(_temp.get_middle_value());
    Serial.println(F(" *C"));
#endif    

#if LCD_TYPE == LCD1602
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Temperature is"));
    lcd.setCursor(0,1);
    lcd.print(_temp.get_middle_value());
    lcd.print(F(" *C"));
#endif
            
    TaskManager::SetTask_(compare_Temp,_SCREEN_DELAY);
  }

  void compare_Temp(void)
  {
#if LCD_TYPE == LCD1602    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Temperature is"));
    lcd.setCursor(0,1);
#endif

#if UART_ENABLED == 1  
    Serial.print(F("Temperature is "));
#endif

    if (_temp.get_value_state() == Serial_measure::Value_state::falling)
    {
      #if LCD_TYPE == LCD1602  
        lcd.print(F("falling "));
      #endif
      
      #if UART_ENABLED == 1  
        Serial.print(F("falling "));
      #endif
    }

    else if (_temp.get_value_state() == Serial_measure::Value_state::rising)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(F("rising "));
      #endif

      #if UART_ENABLED == 1 
        Serial.print(F("rising "));
      #endif
    }

    else if (_temp.get_value_state() == Serial_measure::Value_state::stable)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(F("stable "));
      #endif
      
      #if UART_ENABLED == 1 
        Serial.print(F("stable "));
      #endif
    }

    else {TaskManager::SetTask_(print_meteo_Press,0); return;}

    #if LCD_TYPE == LCD1602 
      lcd.print(_temp.get_state_not_change_times());
      lcd.print(F("min"));
    #endif
      
    #if UART_ENABLED == 1 
      Serial.print(_temp.get_state_not_change_times());
      Serial.println(F("min"));
    #endif
    
    TaskManager::SetTask_(print_meteo_Press,_SCREEN_DELAY);
  }
//////PRINT Pressure/////////
  void  print_meteo_Press(void)
  {
    #if UART_ENABLED == 1 
      Serial.print(F("Pressure is "));
      Serial.print(_press.get_middle_value());
      Serial.println(F(" hPa"));
    #endif

    #if LCD_TYPE == LCD1602 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Pressure is "));
      lcd.setCursor(0,1);
      lcd.print(_press.get_middle_value());
      lcd.print(F(" hPa"));
    #endif
    
    TaskManager::SetTask_(compare_Press,_SCREEN_DELAY);
  }

  void compare_Press(void)
  {
    #if LCD_TYPE == LCD1602 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Pressure is "));
      lcd.setCursor(0,1);
    #endif

    #if UART_ENABLED == 1 
      Serial.print(F("Pressure is "));
    #endif

    if (_press.get_value_state() == Serial_measure::Value_state::falling)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(F("falling "));
      #endif

      #if UART_ENABLED == 1
        Serial.print(F("falling "));
      #endif
    }

    else if (_press.get_value_state() == Serial_measure::Value_state::rising)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(F("rising "));
      #endif

      #if UART_ENABLED == 1
        Serial.print(F("rising "));
      #endif
    }

    else if (_press.get_value_state() == Serial_measure::Value_state::stable)
    {
      #if LCD_TYPE == LCD1602 
        lcd.print(F("stable "));
      #endif
      
      #if LCD_TYPE == LCD1602
        Serial.print(F("stable ")); 
      #endif   
    }
    
    else {TaskManager::SetTask_(print_meteo_Hum,0); return;}

    #if LCD_TYPE == LCD1602
      lcd.print(_press.get_state_not_change_times());
      lcd.print(F("min"));
    #endif

    #if UART_ENABLED == 1
      Serial.print(_press.get_state_not_change_times());
      Serial.println(F("min"));
    #endif
    
    TaskManager::SetTask_(print_meteo_Hum,_SCREEN_DELAY);
  }
  
//////PRINT humidity/////////
  void  print_meteo_Hum(void)
  {
   #if UART_ENABLED == 1
    Serial.print(F("Humidity is "));
    Serial.print(_hum.get_middle_value());
    Serial.println(F(" %"));
   #endif
   
   #if LCD_TYPE == LCD1602
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print(F("Humidity is"));
     lcd.setCursor(0,1);
     lcd.print(_hum.get_middle_value());
     lcd.print(F(" %"));
   #endif 
       
   TaskManager::SetTask_(compare_Hum,_SCREEN_DELAY);
  }

  void compare_Hum(void)
  {
    #if LCD_TYPE == LCD1602
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Humidity is "));
      lcd.setCursor(0,1);
    #endif

    #if UART_ENABLED == 1
      Serial.print(F("Humidity is "));
    #endif

    if (_hum.get_value_state() == Serial_measure::Value_state::falling)
    {
      #if LCD_TYPE == LCD1602
        lcd.print(F("falling "));
      #endif

      #if UART_ENABLED == 1
        Serial.print(F("falling "));
      #endif
    }

    else if (_hum.get_value_state() == Serial_measure::Value_state::rising)
    {
      #if LCD_TYPE == LCD1602
        lcd.print(F("rising "));
      #endif

      #if UART_ENABLED == 1
        Serial.print(F("rising "));
      #endif
    }

    else if (_hum.get_value_state() == Serial_measure::Value_state::stable)
    {
      #if LCD_TYPE == LCD1602
        lcd.print(F("stable "));
      #endif

      #if UART_ENABLED == 1
        Serial.print(F("stable "));   
      #endif
    }
    
    else {TaskManager::SetTask_(device_sleep_,0); return;}
    
    #if LCD_TYPE == LCD1602
      lcd.print(_hum.get_state_not_change_times());
      lcd.print(F("min"));
    #endif
      
    #if UART_ENABLED == 1
      Serial.print(_hum.get_state_not_change_times());
      Serial.println(F("min"));
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
