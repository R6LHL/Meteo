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
    
#ifdef EXTERNAL_SENSOR    
#if EXTERNAL_SENSOR == DS18B20
  ext_temp_sens.write(0x44,0); // start conversion with full power supply
#endif
#endif
    
    meteo_sensor.takeForcedMeasurement();    
     
    TaskManager::SetTask_(collect_meteo, 750);
  }
  
 void collect_meteo(void)
  {
    
    
    _temp.set_new_value(meteo_sensor.readTemperature());
    _press.set_new_value(meteo_sensor.readPressure()/100.0F);
    _hum.set_new_value(meteo_sensor.readHumidity());
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
    Serial.print(F("Temperature is "));
    Serial.print(_temp.get_middle_value());
    Serial.println(F(" *C"));
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Temperature is"));
    lcd.setCursor(0,1);
    lcd.print(_temp.get_middle_value());
    lcd.print(F(" *C"));
            
    TaskManager::SetTask_(compare_Temp,_SCREEN_DELAY);
  }

  void compare_Temp(void)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Temperature is"));
    lcd.setCursor(0,1);

    Serial.print(F("Temperature is "));

    if (_temp.get_value_state() == Serial_measure::Value_state::falling)
    {
      lcd.print(F("falling "));
      Serial.print(F("falling "));
    }

    else if (_temp.get_value_state() == Serial_measure::Value_state::rising)
    {
      lcd.print(F("rising "));
      Serial.print(F("rising "));
    }

    else if (_temp.get_value_state() == Serial_measure::Value_state::stable)
    {
      lcd.print(F("stable "));
      Serial.print(F("stable "));
    }

    else {TaskManager::SetTask_(print_meteo_Press,0); return;}

    lcd.print(_temp.get_state_not_change_times());
    lcd.print(F("min"));
    Serial.print(_temp.get_state_not_change_times());
    Serial.println(F("min"));
    
    TaskManager::SetTask_(print_meteo_Press,_SCREEN_DELAY);
  }
//////PRINT Pressure/////////
  void  print_meteo_Press(void)
  {
    Serial.print(F("Pressure is "));
    Serial.print(_press.get_middle_value());
    Serial.println(F(" hPa"));

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Pressure is "));
    lcd.setCursor(0,1);
    lcd.print(_press.get_middle_value());
    lcd.print(F(" hPa"));
    
    TaskManager::SetTask_(compare_Press,_SCREEN_DELAY);
  }

  void compare_Press(void)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Pressure is "));
    lcd.setCursor(0,1);

    Serial.print(F("Pressure is "));

    if (_press.get_value_state() == Serial_measure::Value_state::falling)
    {
      lcd.print(F("falling "));
      Serial.print(F("falling "));
    }

    else if (_press.get_value_state() == Serial_measure::Value_state::rising)
    {
      lcd.print(F("rising "));
      Serial.print(F("rising "));
    }

    else if (_press.get_value_state() == Serial_measure::Value_state::stable)
    {
      lcd.print(F("stable "));
      Serial.print(F("stable "));    
    }
    
    else {TaskManager::SetTask_(print_meteo_Hum,0); return;}
    
    lcd.print(_press.get_state_not_change_times());
    lcd.print(F("min"));
    Serial.print(_press.get_state_not_change_times());
    Serial.println(F("min"));
    
    TaskManager::SetTask_(print_meteo_Hum,_SCREEN_DELAY);
  }
  
//////PRINT humidity/////////
  void  print_meteo_Hum(void)
  {
   Serial.print(F("Humidity is "));
   Serial.print(_hum.get_middle_value());
   Serial.println(F(" %"));
   
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Humidity is"));
    lcd.setCursor(0,1);
    lcd.print(_hum.get_middle_value());
    lcd.print(F(" %"));
       
   TaskManager::SetTask_(compare_Hum,_SCREEN_DELAY);
  }

  void compare_Hum(void)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Humidity is "));
    lcd.setCursor(0,1);

    Serial.print(F("Humidity is "));

    if (_hum.get_value_state() == Serial_measure::Value_state::falling)
    {
      lcd.print(F("falling "));
      Serial.print(F("falling "));
    }

    else if (_hum.get_value_state() == Serial_measure::Value_state::rising)
    {
      lcd.print(F("rising "));
      Serial.print(F("rising "));
    }

    else if (_hum.get_value_state() == Serial_measure::Value_state::stable)
    {
      lcd.print(F("stable "));
      Serial.print(F("stable "));    
    }
    
    else {TaskManager::SetTask_(device_sleep_,0); return;}
    
    lcd.print(_hum.get_state_not_change_times());
    lcd.print(F("min"));
    Serial.print(_hum.get_state_not_change_times());
    Serial.println(F("min"));
    
    TaskManager::SetTask_(device_sleep_,_SCREEN_DELAY);
  }

  void device_sleep_(void)
  {
    lcd.clear();
    lcd.noBacklight();
    device_sleep = true;

     TaskManager::SetTask_(read_meteo, _SENSOR_ASK_DELAY_MS);
     TaskManager::DeleteTask_(print_meteo_Temp);

    noInterrupts();
    PCICR |= (1<<PCIE0);
    EIMSK |= (1<<INT0);
    interrupts();
  }

 void device_wake(void)
 {
  lcd.backlight();
  device_sleep = false;
  TaskManager::DeleteTask_(read_meteo);
  TaskManager::SetTask_(print_meteo_Temp,0);
 }

 //////////////////////////////////////////////////////



#endif
