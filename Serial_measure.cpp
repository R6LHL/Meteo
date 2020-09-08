#include "Serial_measure.h"

Serial_measure::Serial_measure(void)
{
	iterator = 0;
	first_add = true;
}

void Serial_measure::add_measure(T m)
{
	if (first_add == true)
	{
		for (unsigned char i = 0; i < E; i++)
		{
			measures[i] = m;
		}
		first_add = false;
	}
	
	measures[iterator] = m;
	delta = measures[iterator] - measures[0];
	iterator++;
	
	if (iterator == (E-1)) 
	{
		T last_value = measures[iterator];
		iterator = 0;
		first_add = true;
		
		for (unsigned char i = 1; i < (E-1); i++)
		{
			measures[i] = last_value;
		}
	}
}

void Serial_measure::mov_measure(T m, unsigned char i)
{
	if (i >= (E-1)) return;
	else
	{
		measures[i] = m;
	}
}

T Serial_measure::get_mid_value(void)
{
	T sum;
	if (iterator == 0) return 0;
	for (i = 0; i < iterator; i++)
	{
		sum += measures[i];
	}
	 return  (sum / iterator);
}

T Serial_measure::get_delta(void)
{
	return delta;
}
