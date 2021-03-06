#ifndef __CONFIG_H_
#define __CONFIG_H_
///////////////////////////////////////////////////////////////////////////////////
#define T_TASK_QUEUE_SIZE  (10)	//for Task Manager
#define CPU_CLOCK (16000000) 	// Hz

const unsigned int _SCREEN_DELAY = (2000); //task manager delay in ms between screens
const unsigned int _SLEEP_TIME  = (60000 - (6 * _SCREEN_DELAY));

const unsigned char _SERIAL_MEASURE_QUANTITY = (10);
const unsigned int _SENSOR_ASK_DELAY_MS = (60000);

const unsigned char TEMP_EXT_BUTTON = (8);
//#define TEMP_EXT_BUTTON_CODE          (0b11111110)
const unsigned char TEMP_INT_BUTTON = (9);
//#define TEMP_INT_BUTTON_CODE          (0b11111101)
const unsigned char PRESSURE_BUTTON = (7);
//#define PRESSURE_BUTTON_CODE          (0b11111011)
const unsigned char HUMIDITY_BUTTON = (11);
//#define HUMIDITY_BUTTON_CODE          (0b11110111)
const unsigned char TIME_BATT_BUTTON = (12);
//#define TIME_BATT_BUTTON_CODE         (0b11101111)
//#define KEY_MASK                      (0b00011111)

///////////////////////////////////////////////////////////////////////////////////
#define TEXT_LANGUAGE (ENG)
//#define TEXT_LANGUAGE (RUS)

////////////////////////////////////////////////////////////////////////////////////
#define LCD_AVAILABLE (1)

#if LCD_AVAILABLE == 1
  #define LCD_TYPE          (LCD1602)
  #define LCD_INTERFACE     (I2C)
  const unsigned char LCD_INITIAL_CODE = (0x27);
  const unsigned char LCD_CHARS        = (16);
  const unsigned char LCD_STRINGS      = (2);
#endif

/////////////////////////////////////////////////////////////////////////////////////
#define UART_ENABLED (1)
#if UART_ENABLED == 1
  #define UART_MODE (RX) //or TRX if you need a transmission
  const unsigned int UART_SPEED  = (9600);
#endif


/////////////////////////////////////////////////////////////////////////////////////
#define EXTERNAL_SENSOR (DALLAS_DS18B20) //comment this string if no external sensor present

#if EXTERNAL_SENSOR == DALLAS_DS18B20
  const unsigned char ONE_WIRE_PIN = (3);
  const unsigned int DS18B20_FORCED_DELAY =  (1000);
  const unsigned char FROST_THRESHOLD = 60; // percent
  const float SUMMER_WINTER_THRESHOLD = 20.0;
  #define BYTE_ITERATOR (9)
#endif

/////////////////////////////////////////////////////////////////////////////////////
#define INTERNAL_SENSOR (BME280)

#if INTERNAL_SENSOR == (BME280)
  const float SEALEVELPRESSURE_HPA = (1013.25); // for BME280
  const float LOCAL_27M_PRESSURE_HPA = (1010.01);
  const unsigned char INTERNAL_SENSOR_FORCED_DELAY = 14; //ms 
  const float INTERNAL_TEMP_MIN = 21.0; // degreeC
  const float INTERNAL_TEMP_MAX = 25.0; // degreeC
  const float PRESSURE_MIN_HPA = 1000.0; // hPa
  const float LOCAL_PRESSURE_SUB = LOCAL_27M_PRESSURE_HPA - PRESSURE_MIN_HPA;
  const float FALLOUT_THRESHOLD = 60.0;//%
  const float HUMIDITY_LOW = 35.0;
  const float SUMMER_HUMIDITY_HIGH = 60.0;
  const float WINTER_HUMIDITY_HIGH = 45.0;
#endif

///////////////////////////////////////////////////////////////////////////////////////
#define REAL_TIME_CLOCK (DS3231_)

///////////////////////////////////////////////////////////////////////////////////////
#define POWER_SUPPLY (AUTONOMOUS)

#if POWER_SUPPLY == AUTONOMOUS
  #define AREF_VOLTAGE  (5.0)
  #define SUPPLY_VOLTAGE_ANALOG_PIN (A0)
  #define BATT_VOLTAGE_DIVIDER (1.0)
  #define BATT_VOLTAGE_MEASURED (2.81)
  #define BATT_VOLTAGE_MCU      (2.64)
  #define CALIBRATION_ADDITIVE (2.81-2.64)
  #define BATT_CONTROL_PERIOD_MS (60000)
  #define BATTERY_HIGH (4.14) 
  #define BATTERY_LOW  (3.0)
#endif

//#define DEBUG_MODE

#endif //_CONFIG_H
