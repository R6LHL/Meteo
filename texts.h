#ifndef __TEXTS_H__
#define __TEXTS_H__

	#if TEXT_LANGUAGE == ENG
    #if INTERNAL_SENSOR == BME280
      #define TEXT_BME280_SELFTEST    (F("BME280 self test."))
      #define TEXT_BME280_NOT_FOUND   (F("BME280 Not found!"))
    #endif

	#endif
	
#endif
