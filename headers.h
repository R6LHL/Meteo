#ifndef __HEADERS_H_
#define __HEADERS_H_

#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "config.h"

#include <Serial_measure.h>

#include <TaskManager_.h>

#ifdef EXTERNAL_SENSOR
#if EXTERNAL_SENSOR == DS18B20
  #include <OneWire.h>
#endif
#endif


#endif
