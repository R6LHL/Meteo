#ifndef __HEADERS_H_
#define __HEADERS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "config.h"

#if LCD_AVAILABLE == 1
  #if LCD_INTERFACE == I2C
    #include <LiquidCrystal_I2C.h>
  #endif
#endif

#if INTERNAL_SENSOR == BME280
  #include <Adafruit_BME280.h>
#endif

#ifdef EXTERNAL_SENSOR
  #if EXTERNAL_SENSOR == DS18B20
    #include <OneWire.h>
  #endif
#endif

#ifdef REAL_TIME_CLOCK
  #if REAL_TIME_CLOCK == DS3231_
    #include <DS3231.h>
  #endif
#endif

#include "TaskManager.h"
#include "Serial_measure.h"
#include "texts.h"
/*
  typedef Serial_measure <float,10> Serial_measure_f0_10;
  typedef Serial_measure <float,6> Serial_measure_f10_60;
  typedef Serial_measure <float,24> Serial_measure_f60_1440;
  typedef Serial_measure <float,7> Serial_measure_f1440_10080;
*/
  typedef Serial_measure <float, 10> Serial_measure_f0_10;
  typedef Serial_measure <float, 6> Serial_measure_f10_60;
  //typedef Serial_measure <float, 3> Serial_measure_f60_180;
  //typedef Serial_measure <float, 6> Serial_measure_f180_360;
  //typedef Serial_measure <float, 12> Serial_measure_f360_720;
  typedef Serial_measure <float, 24> Serial_measure_f60_1440;
  typedef Serial_measure <float, 7> Serial_measure_f1440_10080;
/*
  typedef Serial_measure <float, 10> Serial_measure_i0_10;
  typedef Serial_measure <float, 6> Serial_measure_i10_60;
  typedef Serial_measure <float, 24> Serial_measure_i60_1440;
  typedef Serial_measure <float, 7> Serial_measure_i1440_10080;
*/
/*
  typedef Serial_measure <char, 10> Serial_measure_c0_10;
  typedef Serial_measure <char, 6> Serial_measure_c10_60;
  typedef Serial_measure <char, 24> Serial_measure_c60_1440;
  typedef Serial_measure <char, 7> Serial_measure_c1440_10080;
*/


#endif
