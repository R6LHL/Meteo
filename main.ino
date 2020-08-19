
#include "headers.h"
///////////////////////////////////////////////////////
/* SYSTEM ERROR CODES
 *  0 - system ok
 *  1 - extrenal sensor type is wrong
 */
////////////////////////////////////////////////////////
//variables

Adafruit_BME280 meteo_sensor;
LiquidCrystal_I2C lcd(0x27,16,2);  // Устанавливаем дисплей
Serial_measure _temp;
Serial_measure _press;
Serial_measure _hum;

byte system_error_code = 0;

#ifdef DS18B20
OneWire ext_temp_sens(EXTERNAL_SENSOR_PIN);
byte ext_temp_addr[8];
byte ext_temp_data[9];
#endif

bool device_sleep = false;

///////////////////////////////////////////////////////
//Tasks

#include "tasks.h"

///////////////////////////////////////////////////////
//interrupt handlers

ISR(TIMER2_OVF_vect)
{
  //Serial.println("Interrupt is working");
  TaskManager::TimerTaskService_();
}

ISR(INT0_vect)
{
  PCICR &= ~(1<<PCIE0);
  EIMSK &= ~(1<<INT0);

  TaskManager::SetTask_(device_wake,0);
}

void setup()
{
  noInterrupts();
 
  TCCR2B |= (1<<CS22); // (clk/64)
  TIMSK2 |= (1<<TOIE2); // ovf interrupt enabled

#ifdef DS18B20
ext_temp_sens.search(ext_temp_addr); //searching address of ds18b20
if (ext_temp_addr[0] == 0x28) //check sensor type
{
  ext_temp_sens.reset();
  ext_temp_sens.select(ext_temp_addr);
  ext_temp_sens.write(0x44,0); // start conversion with full power supply
}
else {system_error_code = 1;}
}
#endif

  interrupts();

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WAKE_BUTTON,INPUT);
  digitalWrite(WAKE_BUTTON, HIGH);
  
 Serial.println(F("BME280 Sensor event test"));

  if (!meteo_sensor.begin(0x76)) 
   {                                // Инициализация датчика BME280
     Serial.println(F("Could not find a valid BME280!"));  // Печать сообщения об ошибки
     while (1);
   }
 
  
  Serial.begin(9600);

    meteo_sensor.setSampling(Adafruit_BME280::MODE_FORCED, /* Режим работы */
    Adafruit_BME280::SAMPLING_X2, /* Температура передискретизация */
    Adafruit_BME280::SAMPLING_X16, /* Давление передискретизация */
    Adafruit_BME280::SAMPLING_X1,  /* humidity*/
    Adafruit_BME280::FILTER_X16); /* Фильрация. */
       
  lcd.init();                     
  lcd.backlight();// Включаем подсветку дисплея
  
  Serial.println(F("Reading..."));
  lcd.print(F("Reading..."));

  ///initial run
   for (unsigned char i = 0; i < (_SERIAL_MEASURE_QUANTITY); i++ )
   {
    meteo_sensor.takeForcedMeasurement();
    delay(14);
    _temp.set_new_value(meteo_sensor.readTemperature());
    _press.set_new_value(meteo_sensor.readPressure()/100.0F);
    _hum.set_new_value(meteo_sensor.readHumidity());
   }
     /// Working
   TaskManager::SetTask_(read_meteo,0);
}

void loop()
{
  //Serial.println("Queue is processing");
  TaskManager::ProcessTaskQueue_();
}
