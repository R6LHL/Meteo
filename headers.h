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

#include <TaskManager_.h>
#include <Serial_measure.h>

#endif
