#ifndef _INTERRUPTS_H
#define _INTERRUPTS_H

//interrupt handlers

ISR(TIMER2_OVF_vect)
{
  //Serial.println("Interrupt is working");
  TaskManager::TimerTaskService_();
}

/*
ISR(PCINT0_vect)
{
  PCICR &= ~(1<<PCIE0); //Disble interrupt
  /*
  button_pressed = 0b11100000;
  button_pressed |= (digitalRead(TEMP_EXT_BUTTON)<<0);
  button_pressed |= (digitalRead(TEMP_INT_BUTTON)<<1);
  button_pressed |= (digitalRead(PRESSURE_BUTTON)<<2);
  button_pressed |= (digitalRead(HUMIDITY_BUTTON)<<3);
  button_pressed |= (digitalRead(TIME_BATT_BUTTON)<<4);
  //button_pressed &= ~(KEY_MASK);
  */
  /*
   button_pressed = 0;
  if (digitalRead(TEMP_EXT_BUTTON) == LOW) button_pressed = 8;
  if (digitalRead(TEMP_INT_BUTTON) == LOW) button_pressed = 9;
  if (digitalRead(PRESSURE_BUTTON) == LOW) button_pressed = 10;
  if (digitalRead(HUMIDITY_BUTTON) == LOW) button_pressed = 11;
  if (digitalRead(TIME_BATT_BUTTON) == LOW) button_pressed = 12;

#if DEBUG_MODE == ENABLED
    Serial.print(F("BUTTON_CODE"));
    Serial.println(button_pressed,DEC);
#endif 
  
  TaskManager::SetTask_(SYS_device_wake,500);
}

/*
ISR(WDT_OVF_vect)
{
  //DEBUG
  //Serial.println("Interrupt is working");
  //END DEBUG
  
  TaskManager::TimerTaskService_();
}
*/

#endif //_INTERRUPTS_H