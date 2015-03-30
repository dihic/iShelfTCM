#include "ConfigComm.h"


using namespace std;

const std::uint8_t ConfigComm::dataHeader[3] = {0xfe, 0xfc, 0xfd};


ConfigComm::ConfigComm(ARM_DRIVER_USART &u)
	:uart(u)
{
	uart.Initialize(NULL);
	uart.PowerControl(ARM_POWER_FULL);
	uart.Control(ARM_USART_MODE_ASYNCHRONOUS |
							 ARM_USART_DATA_BITS_8 |
							 ARM_USART_PARITY_NONE |
							 ARM_USART_STOP_BITS_1 |
							 ARM_USART_FLOW_CONTROL_NONE, 115200);
}

ConfigComm::~ConfigComm()
{
	Stop();
}

inline void ConfigComm::Start()
{
	uart.Control(ARM_USART_CONTROL_TX, 1);
	uart.Control(ARM_USART_CONTROL_RX, 1);
	dataState = StateDelimiter1;
	command = 0; 
	base = 0;
	dataOffset = data;
}
	
inline void ConfigComm::Stop()
{
	uart.Control(ARM_USART_CONTROL_TX, 0);
	uart.Control(ARM_USART_CONTROL_RX, 0);
}

void ConfigComm::DataReceiver()
{
//	static uint8_t dataState = 0;
//	static uint8_t command = 0; 
//	static uint8_t parameterLen;
//  static uint8_t parameterIndex;
//	static boost::shared_ptr<uint8_t[]> parameters;
//	
//	static uint32_t base = 0;
//	static uint8_t *dataOffset = data;
	
	ARM_USART_STATUS status = uart.GetStatus();
	if (!status.rx_busy)
	{
		uart.Receive(data, 0x400);
		dataOffset = data;
		base = 0;
	}
	
	uint32_t num = uart.GetRxCount() - base;
	if (num == 0)
		return;	
	base+=num;

	for(int i=0; i<num; ++i)
	{
		switch (dataState)
		{
			case StateDelimiter1:
				if (dataOffset[i]==dataHeader[0])
					dataState = StateDelimiter2;
				break;
			case StateDelimiter2:
				if (dataOffset[i] == dataHeader[1])
					dataState = StateCommand;
				else
					dataState = StateDelimiter1;
				break;
			case StateCommand:
				command = dataOffset[i];
				dataState = StateParameterLength;
				break;
			case StateParameterLength:
				parameterLen = dataOffset[i];
				if (parameterLen == 0)
				{
					parameters.reset();
					dataState = StateDelimiter1;
					if (OnCommandArrivalEvent)
						OnCommandArrivalEvent(command, NULL, 0);
				}
				else
				{
					parameters = boost::make_shared<uint8_t[]>(parameterLen);
					parameterIndex = 0;
					dataState = StateParameters;
				}
				break;
			case StateParameters:
				parameters[parameterIndex++] = dataOffset[i];
				if (parameterIndex >= parameterLen)
				{
					if (OnCommandArrivalEvent)
						OnCommandArrivalEvent(command,parameters.get(),parameterLen);
					parameters.reset();
					dataState = StateDelimiter1;
//					uart.Control(ARM_USART_ABORT_RECEIVE,  0);
//					uart.Control(ARM_USART_CONTROL_RX, 1);
//					uart.Receive(data, 0x200);
//					dataOffset = data;
//					base = 0;
//					return;
				}
				break;
			default:
				break;
		}
	}
	dataOffset+=num;
}

bool ConfigComm::SendData(const uint8_t *data, size_t len)
{
	uint8_t pre[3]={ dataHeader[0],dataHeader[2],len };
	ARM_USART_STATUS status;
	do
	{
		status = uart.GetStatus();
	} while (status.tx_busy);
	uart.Send(pre,3);
	do
	{
		status = uart.GetStatus();
	} while (status.tx_busy);
	uart.Send(data,len);
	return true;
}

bool ConfigComm::SendData(uint8_t command,const uint8_t *data,size_t len)
{
	uint8_t pre[4]={ dataHeader[0],dataHeader[2],command,len };
	ARM_USART_STATUS status;
	do
	{
		status = uart.GetStatus();
	} while (status.tx_busy);
	uart.Send(pre,4);
	if (len>0)
	{
		do
		{
			status = uart.GetStatus();
		} while (status.tx_busy);
		uart.Send(data,len);
	}
	return true;
}


