#ifndef __CONFIG_H_
#define __CONFIG_H_

#define T_TASK_QUEUE_SIZE (5)	//for Task Manager
#define CPU_CLOCK (16000000) 	// Hz

#define _SCREEN_DELAY (2000) //task manager delay in ms between screens
#define _SLEEP_TIME (60000 - (6 * _SCREEN_DELAY))

#define _SERIAL_MEASURE_QUANTITY (10)
#define _SENSOR_ASK_DELAY_MS (6000)

#define WAKE_BUTTON (2)

////////////////////////////////////////////////////////////////////////////////////
#define LCD_AVAILABLE (1)

#if LCD_AVAILABLE == 1
  #define LCD_TYPE          (LCD1602)
  #define LCD_INTERFACE     (I2C)
  #define LCD_INITIAL_CODE  (0x27)
  #define LCD_CHARS         (16)
  #define LCD_STRINGS       (2)
#endif

/////////////////////////////////////////////////////////////////////////////////////
#define UART_ENABLED (1)

#if UART_ENABLED == 1
  #define UART_SPEED (9600)
#endif

/////////////////////////////////////////////////////////////////////////////////////
#define EXTERNAL_SENSOR (DS18B20) //comment this string if no external sensor present

#if EXTERNAL_SENSOR == DS18B20
  #define EXTERNAL_SENSOR_PIN (10)
  #define DS18B20_FORCED_DELAY (750)
#endif

/////////////////////////////////////////////////////////////////////////////////////
#define INTERNAL_SENSOR (BME280)

#if INTERNAL_SENSOR == (BME280)
  #define _BME280_FORCED_DELAY (14)
  #define SEALEVELPRESSURE_HPA (1013.25) // for BME280
  #define LOCAL_27M_PRESSURE_HPA (1010.01)
  
#endif

///////////////////////////////////////////////////////////////////////////////////////

#endif
