#ifndef __TEXTS_H__
#define __TEXTS_H__

	#if TEXT_LANGUAGE == ENG
    #if INTERNAL_SENSOR == BME280
      #define TEXT_BME280_SELFTEST    (F("BME280 self test."))
      #define TEXT_BME280_NOT_FOUND   (F("BME280 Not found!"))
     
    #endif
    
    #define TEXT_TEMPERATURE_IS       (F("Temperature is "))
    #define TEXT_CELSIUS_DEGREE       (F(" *C"))
    #define TEXT_FALLING              (F("falling "))
    #define TEXT_RISING               (F("rising "))
    #define TEXT_STABLE               (F("stable "))
    #define TEXT_MIN                  (F("min"))
    
    #define TEXT_PRESSURE_IS          (F("Pressure is "))
    #define TEXT_HPA                  (F(" hPa"))

    #define TEXT_HUMIDITY_IS          (F("Humidity is "))
    #define TEXT_PERCENT_SIGN         (F(" %"))

    #define TEXT_SYSTEM_OK            (F("SYSTEM OK "))
    #define TEXT_SYSTEM_ERROR         (F("SYSTEM ERROR "))

    #define TEXT_ROOM_TEMPERATURE     (F("Room temp-re "))
    
	#endif
	
#endif
