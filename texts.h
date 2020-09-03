#ifndef __TEXTS_H__
#define __TEXTS_H__

	#if TEXT_LANGUAGE == ENG
    #if INTERNAL_SENSOR == BME280
      //#define TEXT_BME280_SELFTEST    (F("BME280 self test."))
      #define TEXT_BME280_NOT_FOUND   (F("BME280 Not found!\0"))
     
    #endif
    
    #define TEXT_TEMPERATURE_IS       (F("Temperature is \0"))
    #define TEXT_CELSIUS_DEGREE       (F(" *C\0"))
    #define TEXT_FALLING              (F("falling \0"))
    #define TEXT_RISING               (F("rising \0"))
    #define TEXT_STABLE               (F("stable \0"))
    #define TEXT_MIN                  (F("min\0"))
    
    #define TEXT_PRESSURE_IS          (F("Pressure is \0"))
    #define TEXT_HPA                  (F(" hPa\0"))

    #define TEXT_HUMIDITY_IS          (F("Humidity is \0"))
    #define TEXT_PERCENT_SIGN         (F(" %\0"))

    #define TEXT_SYSTEM_OK            (F("SYSTEM OK \0"))
    #define TEXT_SYSTEM_ERROR         (F("SYSTEM ERROR \0"))

    #define TEXT_ROOM_TEMPERATURE     (F("Room temp-re \0"))
    #define DATE_DEVIDER              (F("-\0"))
    #define TIME_DEVIDER              (F(":\0"))
    #define SPACE_DEVIDER             (F(":\0"))
    /*
    #define TEXT_SET_DATE             (F("setdate\0"))
    //const char TEXT_SET_DATE []PROGMEM = ("setdate \0");
    #define TEXT_SET_TIME             (F("settime\0"))
    //const char TEXT_SET_TIME []PROGMEM = ("settime \0");
    #define TEXT_SET_DOW              (F("setdow\0"))
    //const char TEXT_SET_DOW []PROGMEM = ("setdow \0");
    */
    
	#endif
	
#endif
