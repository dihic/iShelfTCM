#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE

#include "System.h"
#include "CanEx.h"
#include "fram.h"
#include "UnitManager.h"
#include "NetworkConfig.h"
#include "NetworkEngine.h"

#include "FastDelegate.h"

using namespace fastdelegate;
using namespace IntelliStorage;
using namespace std;

boost::shared_ptr<CANExtended::CanEx> CanEx;
boost::shared_ptr<NetworkConfig> ethConfig;
boost::shared_ptr<NetworkEngine> ethEngine;
boost::shared_ptr<FRAM> fram;

UnitManager unitManager;

void SystemHeartbeat(void const *argument)
{
	static uint8_t hbcount = 20;
	
	HAL_GPIO_TogglePin(STATUS_PIN);
	
	//Send a heart beat per 10s
	if ((ethEngine.get()!=NULL) && ++hbcount>20)
	{
		hbcount = 0;
 		ethEngine->SendHeartBeat();
	}
}
osTimerDef(TimerHB, SystemHeartbeat);

void HeartbeatArrival(uint16_t sourceId, CANExtended::DeviceState state)
{
#ifdef DEBUG_PRINT
	static int dc =0;
#endif
	if (state != CANExtended::Operational)
		return;
	CanEx->Sync(sourceId, SYNC_LIVE, CANExtended::Trigger); //Confirm & Stop
	if (sourceId & 0x100)
	{
		boost::shared_ptr<StorageUnit> unit = unitManager.FindUnit(sourceId);
		if (unit.get() == NULL)
		{
#ifdef DEBUG_PRINT
			cout<<"#"<<++dc<<" DeviceID: "<<(sourceId & 0xff)<<" Added"<<endl;
#endif
			unit.reset(new StorageUnit(*CanEx, sourceId));
			CanEx->AddDevice(unit);
			unit->ReadCommandResponse.bind(ethEngine.get(), &NetworkEngine::DeviceReadResponse);
			unit->WriteCommandResponse.bind(ethEngine.get(), &NetworkEngine::DeviceWriteResponse);
			unitManager.Add(sourceId, unit);
		}
	}
}

static void UpdateWorker (void const *argument)  
{
	while(1)
	{
		ethConfig->Poll();
		CanEx->Poll();
		osThreadYield();
	}
}
osThreadDef(UpdateWorker, osPriorityNormal, 1, 0);

static void UpdateUnits(void const *argument)  //Prevent missing status
{
	osDelay(2000);
	std::map<std::uint16_t, boost::shared_ptr<StorageUnit> > &unitList = unitManager.GetList();
//	while(1)
//	{
		//CanEx->SyncAll(SYNC_DATA, CANExtended::Trigger);
		for(UnitManager::UnitIterator it = unitList.begin(); it!= unitList.end(); ++it)
		{
			if (!it->second->IsBusy())
			{
				CanEx->Sync(it->second->DeviceId, SYNC_DATA, CANExtended::Trigger);
				osDelay(20);
			}
		}
	while(1)
	{
		osDelay(250);
		if (ethEngine.get()!=NULL)
			ethEngine->InventoryRfid();
	}
}
osThreadDef(UpdateUnits, osPriorityNormal, 1, 0);

int main()
{
	HAL_Init();		/* Initialize the HAL Library    */
	
	CommStructures::Register();
	
	cout<<"Started..."<<endl;
	
	//Initialize F-Ram and config UART
	osDelay(100);	//Wait for voltage stable
	fram = FRAM::CreateInstance(Driver_SPI2);
	ethConfig.reset(new NetworkConfig(*fram, Driver_USART1));
	
	//Initialize CAN
	CanEx.reset(new CANExtended::CanEx(Driver_CAN1, 0x001));
	CanEx->HeartbeatArrivalEvent.bind(&HeartbeatArrival);
#ifdef DEBUG_PRINT
	cout<<"CAN Inited"<<endl;
#endif
	
	//Initialize Ethernet interface
	net_initialize();
	
	ethEngine.reset(new NetworkEngine(ethConfig->GetIpConfig(IpConfigGetServiceEnpoint), unitManager.GetList()));
	ethConfig->ServiceEndpointChangedEvent.bind(ethEngine.get(),&NetworkEngine::ChangeServiceEndpoint);
//	unitManager.OnSendData.bind(ethEngine.get(),&NetworkEngine::SendRfidData);
	
#ifdef DEBUG_PRINT
	cout<<"Ethernet SPI Speed: "<<Driver_SPI1.Control(ARM_SPI_GET_BUS_SPEED, 0)<<endl;
	cout<<"Ethernet Inited"<<endl;
#endif

	//Start to find unit devices
	CanEx->SyncAll(SYNC_LIVE, CANExtended::Trigger);
	
	//Initialize system heatbeat
	osTimerId id = osTimerCreate(osTimer(TimerHB), osTimerPeriodic, NULL);
  osTimerStart(id, 500); 
	
	osThreadCreate(osThread(UpdateWorker), NULL);
	osThreadCreate(osThread(UpdateUnits), NULL);
	
  while(1) 
	{
		net_main();
    ethEngine->Process();
		//osThreadYield();
  }
}
