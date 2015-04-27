#ifndef __SENSOR_ARRAY_H 
#define __SENSOR_ARRAY_H

#include "hx711.h"

#define PRECISION_DIGITS 14

#define SENSOR1_SCK  PORT0,2
#define SENSOR1_DOUT PORT0,1
#define SENSOR2_SCK  PORT0,4
#define SENSOR2_DOUT PORT0,3
#define SENSOR3_SCK  PORT1,1
#define SENSOR3_DOUT PORT1,0
#define SENSOR4_SCK  PORT1,5
#define SENSOR4_DOUT PORT1,4

class SensorArray
{
	public:
		HX711 *pSensors[4];
		void Setup();
		void Refresh();
};

#endif
