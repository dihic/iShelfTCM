#include "SensorArray.h"
#include "gpio.h"

static HX711 sensor1;
static HX711 sensor2;
static HX711 sensor3;
static HX711 sensor4;

void SensorArray::Setup()
{
	sensor1.Setup(SENSOR1_SCK,SENSOR1_DOUT);
	sensor2.Setup(SENSOR2_SCK,SENSOR2_DOUT);
	sensor3.Setup(SENSOR3_SCK,SENSOR3_DOUT);
	sensor4.Setup(SENSOR4_SCK,SENSOR4_DOUT);
	pSensors[0]=&sensor1;
	pSensors[1]=&sensor2;
	pSensors[2]=&sensor3;
	pSensors[3]=&sensor4;
}

void SensorArray::Refresh()
{
	if (GPIOIntStatus(SENSOR1_DOUT))
		pSensors[0]->ReadData(PRECISION_DIGITS);
	if (GPIOIntStatus(SENSOR2_DOUT))
		pSensors[1]->ReadData(PRECISION_DIGITS);
	if (GPIOIntStatus(SENSOR3_DOUT))
		pSensors[2]->ReadData(PRECISION_DIGITS);
	if (GPIOIntStatus(SENSOR4_DOUT))
		pSensors[3]->ReadData(PRECISION_DIGITS);
}
