#ifndef _SERIAL_MEASURE_H
#define _SERIAL_MEASURE_H

#ifndef   _SERIAL_MEASURE_QUANTITY 
#define   _SERIAL_MEASURE_QUANTITY  (10)
#endif

#include <math.h>

class Serial_measure
{
	public:
	
	Serial_measure(void);
	
	int get_middle_value(void);
	float get_current_value(void);
	float get_previous_value(void);
	char get_value_state(void);
	char get_value_state_prev(void);
	int get_state_not_change_times(void);
	
	void set_new_value(float);
	
	enum Value_state
	{
	 falling = -1,
	 stable = 0,
	 rising = 1,
	 not_defined = 2,
	};
	
	
	private:
	
	float measure_array[_SERIAL_MEASURE_QUANTITY];
	unsigned char number_of_measure;
	char value_state;
	char value_state_prev;
	
	bool is_first_start;
	
	int state_not_changing_times;
	
	float middle_value;
	float middle_value_prev;
	
	void calc_middle_value(void);
	void is_state_changed(void);
};

#endif