#ifndef _SERIAL_MEASURE_H
#define _SERIAL_MEASURE_H

template <typename T, char E>
class Serial_measure
{
	public:
	
	Serial_measure(void)
  {
    iterator = 0;
    first_add = true;
	carry_flag = false;
  }
	
	//void add_measure(T);
	//void mov_measure(T, unsigned char);
	//T get_mid_value(void);
	//T get_delta(void);
	unsigned char get_iterator(void) {return iterator;}
	
	T get_measure(unsigned char i){return measures[i];}
	bool get_carry_flag(void){return carry_flag;}
	void set_carry_flag(bool f){carry_flag = f;}
	
	T get_delta(void)
	{
		return measures[iterator] - measures[0];
	}
	
	T get_delta(unsigned char element, unsigned char range)
	{
		unsigned char element1;
		if((element - range) < 0)
		{
			element1 = E - abs(element - range);
		}
		else {element1 = element - range;}
		return measures[element] - measures[element1];
	}
/*
void add_measure(T m)
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
*/
void mov_measure(T m, unsigned char i)
{

  if (i >= (E))
  {
    //carry_flag = true;
	iterator = 0;
  }
  else
  {
    measures[i] = m;
    iterator = i;
	//carry_flag = false;
  }
  
  if (first_add == true)
  {
		for (unsigned char j = 0; j < E; j++) 
		{
				measures[j] = m;
		}
		first_add = false;
  }
  
  //delta = measures[iterator] - measures[0];
}

T get_mid_value(void)
{
  long temp = 0;
  
  for (unsigned char i = 0; i < E; i++)
  {
    
	temp = (temp + measures[i]);
  }
 
	
   return  (T)temp/E;
}


	
	private:
	
	T measures[E];
	unsigned char iterator;
	bool first_add;
	bool carry_flag;
  
};

#endif
