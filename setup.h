#ifndef _SETUP_H
#define _SETUP_H

/////////////////////////////////////
void Setup_timers(void){
	
#ifdef DEBUG_MODE  	
	Serial.print(F("Setup timers...\0"));
#endif
	
  TCCR2B |= (1<<CS22); // (clk/64)
  TIMSK2 |= (1<<TOIE2); // ovf interrupt enabled
  
#ifdef DEBUG_MODE  
	Serial.println(F("Done!\0"));
#endif
}
//WDTCSR |= (1<<WDIE)|(1<<WDCE)|(1<<WDP2)|(1<<WDP1); //tune watchdog timer for 1ms interrupt (128kHz/128) as TaskManager Timer

//////////////////////////////////

void Setup_IOpins(void){
	
#ifdef DEBUG_MODE  
	Serial.print(F("Setup IO pins...\0"));
#endif
	
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
  
	pinMode(TEMP_EXT_BUTTON,INPUT_PULLUP);
	pinMode(TEMP_INT_BUTTON,INPUT_PULLUP);
	pinMode(PRESSURE_BUTTON,INPUT_PULLUP);
	pinMode(HUMIDITY_BUTTON,INPUT_PULLUP);
	pinMode(TIME_BATT_BUTTON,INPUT_PULLUP);
	
#ifdef DEBUG_MODE  
	Serial.println(F("Done!\0"));
#endif
}

void Setup_UART(void){
	
#if UART_ENABLED == 1
  Serial.begin(9600);
  
	#ifdef DEBUG_MODE   
	Serial.print(F("Setup UART...\0"));
	#endif
	
  UCSR0B |= (1<<RXCIE0);
  
	#ifdef DEBUG_MODE   
	Serial.println(F("Done!\0"));
	#endif
  
#endif
}

void Setup_LCD(void){
	
#if LCD_AVAILABLE == 1
  #if LCD_TYPE == LCD1602
  
	#ifdef DEBUG_MODE  
	Serial.print(F("Setup LCD...\0"));
	#endif
	
    lcd.init();                   
    lcd.backlight();// Включаем подсветку дисплея
	
	#ifdef DEBUG_MODE   
	Serial.println(F("Done!"));
	#endif
	
  #endif
#endif
}

void Setup_internal_sensor(void){
	
#if INTERNAL_SENSOR == BME280
		
	#ifdef DEBUG_MODE   
	Serial.print(F("Setup BME280...\0"));
	#endif
     
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
	
	#ifdef DEBUG_MODE  
	Serial.println(F("Done!\0"));
	#endif
    
#endif
}

void Setup_RTC(void){
	
#if REAL_TIME_CLOCK == DS3231_

	#ifdef DEBUG_MODE   
	Serial.print(F("Setup DS3231 RTC...\0"));
	#endif
	
	rtc.setClockMode(false); //24h format
  //rtc.setA2Time(1, 1, 1, 0b00011100, bool A2Dy, bool A2h12, bool A2PM); //Alarm once per minute (at seconds = 00)
	
	#ifdef DEBUG_MODE    
	Serial.println(F("Done!\0"));
	#endif
	
#endif
}

#endif //_SETUP_H
