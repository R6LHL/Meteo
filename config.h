#ifndef __CONFIG_H_
#define __CONFIG_H_

#define T_TASK_QUEUE_SIZE (5)	//for Task Manager
#define CPU_CLOCK (16000000) 	// Hz
#define SEALEVELPRESSURE_HPA (1013.25) // for BME280
#define _SCREEN_DELAY (2000) //task manager delay in ms between screens
#define _SLEEP_TIME (60000 - (6 * _SCREEN_DELAY))

#define _SERIAL_MEASURE_QUANTITY 10
#define _SENSOR_ASK_DELAY_MS (6000)

#define _BME280_FORCED_DELAY (14)

#define WAKE_BUTTON (2)

//#define EXTERNAL_SENSOR_ENABLED

#ifdef EXTERNAL_SENSOR_ENABLED
  #define DS18B20
  #define EXTERNAL_SENSOR_PIN (10)
#endif

#endif
