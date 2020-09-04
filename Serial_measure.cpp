#include "Serial_measure.h"

Serial_measure::Serial_measure(void)
{
	number_of_measure = 0;
	value_state = Value_state::not_defined;
	value_state_prev = Value_state::not_defined;
	is_first_start = true;
}
///interface
int Serial_measure::get_middle_value(void)
{
	//calc_middle_value();	
	return middle_value;
}

float Serial_measure::get_current_value(void)
{
	return measure_array[number_of_measure];
}

float Serial_measure::get_previous_value(void)
{
	return measure_array[number_of_measure - 1];
}

char Serial_measure::get_value_state(void)
{
	return value_state;
}

char Serial_measure::get_value_state_prev(void)
{
	return value_state_prev;
}

void Serial_measure::set_new_value(float v)
{
	if (is_first_start == true)
	{
		for (unsigned char i = 0; i < (_SERIAL_MEASURE_QUANTITY-1); ++i)
		{
			measure_array[i] = ceil(v);
		}
			
		is_first_start = false;
		calc_middle_value();
		value_state = Value_state::not_defined;
		value_state_prev = Value_state::not_defined;
		number_of_measure = 0;
		state_not_changing_times = 0;
		calc_middle_value();
		return;
	}
	
	measure_array[number_of_measure] = ceil(v);
	
	if (number_of_measure < (_SERIAL_MEASURE_QUANTITY-1) )
	{
		number_of_measure++;
	}
	else
	{
		number_of_measure = 0;
		calc_middle_value();
	}
}

void Serial_measure::calc_middle_value(void)
{
	middle_value_prev = middle_value;
	
	for (unsigned char i = 0; i < (_SERIAL_MEASURE_QUANTITY - 1); ++i)
	{
		middle_value += measure_array[i];
	}
	
	middle_value = (middle_value)/_SERIAL_MEASURE_QUANTITY;
		
	if (middle_value < middle_value_prev) 
	{
		value_state = Value_state::falling;
	}
	else if (middle_value > middle_value_prev) 
	{
		
		value_state = Value_state::rising;
	}
	else 
	{
		value_state = Value_state::stable;
	}
	
	is_state_changed();
	value_state_prev = value_state;
}

int Serial_measure::get_state_not_change_times(void)
{
	return state_not_changing_times;
}

void Serial_measure::is_state_changed(void)
{
	if (value_state != value_state_prev)
	{
		state_not_changing_times = 0;
	}
	else state_not_changing_times++;
}