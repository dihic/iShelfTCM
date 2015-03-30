#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "stm32f4xx_hal_conf.h"         // Keil::Device:STM32Cube Framework:Classic
#include "Driver_CAN.h"
#include "Driver_SPI.h"                 // ::CMSIS Driver:SPI
#include "Driver_USART.h"               // ::CMSIS Driver:USART

#define STATUS_PIN		GPIOC, GPIO_PIN_13

extern ARM_DRIVER_CAN Driver_CAN1;
extern ARM_DRIVER_SPI Driver_SPI1;
extern ARM_DRIVER_SPI Driver_SPI2;
extern ARM_DRIVER_USART Driver_USART1;

extern RNG_HandleTypeDef RNGHandle;


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */
	
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#define GET_RANDOM_NUMBER HAL_RNG_GetRandomNumber(&RNGHandle)

#endif
