#ifndef _FROST_H
#define _FROST_H

namespace Frost
{

	float temp13 = 0;
	float temp21 = 0;
	float probability;
	
	unsigned const char day_hour = 13;
	unsigned const char night_hour = 21;
	
	void setTemp(float, unsigned char);
		
	unsigned char getProbability(void);
	
	bool state_value_day = false;
	bool state_value_night = false;
}

#endif //Frost

void Frost::setTemp(float t, unsigned char h){
	
	if (h == day_hour){
		temp13 = t;
		state_value_day = true;
	}
	else if (h == 21){
		temp21 = t;
		state_value_night = true;
	}
}

unsigned char Frost::getProbability(void){
	
	float temp_substract = temp13 - temp21;
	double k;
	float offset;
	const float offset100 = 1.75;
	const float offset60 = 4.5;
	const float offset40 = 6.5;
	const float offset20 = 8.75;
	const float offset10 = 11.25;
	
	if (temp21 <= offset100) {
		k = 0.371;
		offset = offset100;
	}
	else if (temp21 > offset100 && temp21 <= offset60){
		
		k = 0.412;
		offset = offset60;
	}
	
	else if (temp21 > offset60 && temp21 <= offset40){
		
		k = 0.412;
		offset = offset40;
	}
	
	else if (temp21 > offset40 && temp21 <= offset20){
		
		k = 0.376;
		offset = offset20;
	}
	
	else if (temp21 > offset60 && temp21 <= offset10){
		
		k = 0.376;
		offset = offset10;
	}
	else return 0.0; //if temp21 >10
	
	probability = (k * temp_substract) + offset;
	probability = (temp21 / probability) * 100;
	
	if (temp21 == 0) probability = 100;
	
	if ((state_value_day == false) || (state_value_night == false)) probability = 50;
	else if ((state_value_day == true) && (state_value_night == true)){
		state_value_day = false;
		state_value_night = false;
	}
	
	if (probability > 100) probability = 100;
	
	return probability;
}