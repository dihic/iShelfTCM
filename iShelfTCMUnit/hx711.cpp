#include <cstdint>
#include <LPC11xx.h>
#include "gpio.h"
#include "hx711.h"
#include "delay.h"

using namespace std;

#define HX_SCK  sckPort,sckBit
#define HX_DOUT doutPort,doutBit

void HX711::Setup(uint32_t sckPortNum, uint32_t sckBitPos,uint32_t doutPortNum, uint32_t doutBitPos)
{
	sckPort=sckPortNum;
	sckBit=sckBitPos;
	doutPort=doutPortNum;
	doutBit=doutBitPos;
	GPIOSetDir(HX_SCK,E_IO_OUTPUT);
	DrvGPIO_ClrBit(HX_SCK);
	GPIOSetDir(HX_DOUT,E_IO_INPUT);
	GPIOSetInterrupt(HX_DOUT,1,0,0);
	GPIOIntEnable(HX_DOUT);
	CurrentData=0;
}

int32_t HX711::ReadData(uint8_t num)
{
	//while (GPIOGetValue(HX_DOUT));
	GPIOIntClear(HX_DOUT);
//		if (GPIOGetValue(HX_DOUT))
//			return 0;
	GPIOIntDisable(HX_DOUT);
	uint32_t val=0;
	for (int8_t i=23;i>=0;i--)
	{
		GPIOSetValue(HX_SCK,1);
		DELAY(10);
		GPIOSetValue(HX_SCK,0);
		if (GPIOGetValue(HX_DOUT))
			val|=(1<<i);
		DELAY(5);
	}
	GPIOSetValue(HX_SCK,1);
	DELAY(10);
	GPIOSetValue(HX_SCK,0);
	DELAY(10);
	val=((~val)&0x007fffffu);
	if (num>0 && num<23)
		val>>=(23-num);
	
	GPIOIntEnable(HX_DOUT);
	
	
	
	if (val & (1<<(num-1)))
	{
		uint32_t mask=0;
		for(int8_t i=0; i<num-1; ++i)
			mask |= (1<<i);
		val = (~val) & mask;
		CurrentData = 0 - val;
	}
	else
		CurrentData=val;
	
	
	return CurrentData;
}			
