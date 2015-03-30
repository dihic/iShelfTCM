#include "fram.h"
#include <cmsis_os.h> 
#include <cstring>

using namespace std;

#define WREN 0x06
#define WRDI 0x04
#define RDSR 0x05
#define WRSR 0x01
#define READ 0x03
#define WRITE 0x02

#define WP_ALL 0x06
#define WP_HALF 0x04
#define WP_QUARTER 0x02
#define WP_NONE 0

#define IDENTIFIER_ADDR 0x00
#define IDENTIFIER_0 0xAA
#define IDENTIFIER_1 0xAB
#define IDENTIFIER_2 0xAC
#define IDENTIFIER_3 0xAD

boost::shared_ptr<FRAM> FRAM::instance;

FRAM::FRAM(ARM_DRIVER_SPI &spi)
	: driver(spi), firstUse(false)
{		
	GPIO_InitTypeDef gpioType = { WP_PIN | HOLD_PIN, 
																GPIO_MODE_OUTPUT_PP,
																GPIO_PULLUP,
																GPIO_SPEED_LOW,
																0 };
	
	HAL_GPIO_Init(FRAM_IO_PORT, &gpioType);
	HAL_GPIO_WritePin(FRAM_IO_PORT, WP_PIN, GPIO_PIN_SET);	//Source WP Set
	HAL_GPIO_WritePin(FRAM_IO_PORT, HOLD_PIN, GPIO_PIN_SET);	//Source HOLD Set
	
	driver.Initialize(NULL);
	driver.PowerControl(ARM_POWER_FULL);
																
	driver.Control(ARM_SPI_MODE_MASTER |
									ARM_SPI_CPOL0_CPHA0 |
									ARM_SPI_DATA_BITS(8) |
									ARM_SPI_MSB_LSB |
									ARM_SPI_SS_MASTER_SW,
									10000000);
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

inline void FRAM::SpiSync()
{
	while (driver.GetStatus().busy)
		__nop();
}

void FRAM::Init()
{
	uint8_t data[4];

	ReadMemory(IDENTIFIER_ADDR,data,4);
	if (data[0]!=IDENTIFIER_0)
	{
		data[0]=IDENTIFIER_0;
		firstUse=true;
	}
	if (data[1]!=IDENTIFIER_1)
	{
		data[1]=IDENTIFIER_1;
		firstUse=true;
	}
	if (data[2]!=IDENTIFIER_2)
	{
		data[2]=IDENTIFIER_2;
		firstUse=true;
	}
	if (data[3]!=IDENTIFIER_3)
	{
		data[3]=IDENTIFIER_3;
		firstUse=true;
	}
	if (firstUse)
		WriteMemory(IDENTIFIER_ADDR, data, 4);
}
	
inline void FRAM::WriteAccess(bool enable)
{
	uint8_t access = enable ? WREN : WRDI;
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	driver.Send(&access, 1);
	SpiSync();
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}
	
uint8_t FRAM::ReadStatus()
{
	uint8_t buf[4] = { RDSR, 0, 0, 0 };
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	driver.Transfer(buf, buf+2, 2);
	SpiSync();
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	return buf[3];
}
	
void FRAM::WriteStatus(uint8_t status)
{
	uint8_t buf[2]={ WRSR, status };
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	driver.Send(buf, 2);
	SpiSync();
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}
	
void FRAM::WriteMemory(uint16_t address, const void *data, uint16_t length)
{
	WriteAccess(true);
	osDelay(1); 
	uint8_t buf[2] = { ((address&0x100)>>5)|WRITE , address & 0xff };
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	driver.Send(buf, 2);
	SpiSync();
	driver.Send(reinterpret_cast<const uint8_t *>(data), length);
	SpiSync();
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}
	
void FRAM::ReadMemory(uint16_t address,void *data,uint16_t length)
{
	uint8_t buf[2] = { ((address&0x100)>>5)|READ, address & 0xff };
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	driver.Send(buf, 2);
	SpiSync();
	driver.Receive(reinterpret_cast<uint8_t *>(data), length);
	SpiSync();
	driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

