#ifndef _FRAM_H
#define _FRAM_H

#include "Driver_SPI.h"
#include "stm32f4xx_hal_gpio.h"

#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#define FRAM_IO_PORT 		GPIOE
#define WP_PIN 					GPIO_PIN_0
#define HOLD_PIN  			GPIO_PIN_1  

class FRAM
{
	private:
		const ARM_DRIVER_SPI &driver;
		bool firstUse;
	
		void WriteAccess(bool enable);
		void WriteStatus(std::uint8_t status);
		std::uint8_t ReadStatus();
		void SpiSync();
		void Init();
		FRAM(ARM_DRIVER_SPI &spi);
		
		static boost::shared_ptr<FRAM> instance;
	public:
		~FRAM() {}
		void WriteMemory(std::uint16_t address, const void *data, std::uint16_t length);
		void ReadMemory(std::uint16_t address, void *data, std::uint16_t length);
		bool IsFirstUse() const { return firstUse; }
			
		static boost::shared_ptr<FRAM> &CreateInstance(ARM_DRIVER_SPI &spi) 
		{ 
			if (instance.get() == NULL)
			{
				instance.reset(new FRAM(spi));
				instance->Init();
			}
			return instance;
		};
};

#endif
