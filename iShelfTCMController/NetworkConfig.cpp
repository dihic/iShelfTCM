#include "NetworkConfig.h"
#include "System.h"
#include <cstring>
#pragma diag_suppress 368
#include <rl_net_lib.h>


using namespace std;

extern ETH_CFG eth0_config;
extern LOCALM nlocalm[];
extern LOCALM localm[];
extern char lhost_name[];

#define MAC_ADDRESS 		 0x10
#define IPCONFIG_ADDRESS 0x16
#define ENDPOINT_ADDRESS 0x2A
	
NetworkConfig::NetworkConfig(FRAM &f, ARM_DRIVER_USART &u)
	:mem(f),comm(u)
{
	comm.OnCommandArrivalEvent.bind(this, &NetworkConfig::CommandArrival);
	Init();
}

void NetworkConfig::Init()
{
	if (mem.IsFirstUse())
	{
		GenerateMacAddress();
		mem.WriteMemory(IPCONFIG_ADDRESS,nlocalm[0].IpAddr,20);
		serviceEndpoint[0]=192;
		serviceEndpoint[1]=168;
		serviceEndpoint[2]=0;
		serviceEndpoint[3]=1;
		serviceEndpoint[4]=0x40;
		serviceEndpoint[5]=0x1F;
		mem.WriteMemory(ENDPOINT_ADDRESS,serviceEndpoint,6);
	}
	else
	{
		mem.ReadMemory(MAC_ADDRESS,eth0_config.MacAddr,6);
		mem.ReadMemory(IPCONFIG_ADDRESS,nlocalm[0].IpAddr,20);
		mem.ReadMemory(ENDPOINT_ADDRESS,serviceEndpoint,6);
	}
	comm.Start();
}

const uint8_t *NetworkConfig::GetIpConfig(IpConfigItem item)
{
	const uint8_t *result = NULL;
	switch (item)
	{
//		case IpConfigSetDHCPEnable:
//			break;
		case IpConfigGetIpAddress:
			result = localm[0].IpAddr;
			break;
		case IpConfigGetMask:
			result = localm[0].NetMask;
			break;
		case IpConfigGetGateway:
			result = localm[0].DefGW;
			break;
		case IpConfigGetServiceEnpoint:
			result = serviceEndpoint;
			break;
		default:
			break;
	}
	return result;
}

void NetworkConfig::SetIpConfig(IpConfigItem item, const uint8_t *data)
{
	switch (item)
	{
//		case IpConfigSetDHCPEnable:
//			break;
		case IpConfigSetIpAddress:
			if (memcmp(localm[0].IpAddr,data,4))
			{
				memcpy(localm[0].IpAddr,data,4);
				mem.WriteMemory(IPCONFIG_ADDRESS,localm[0].IpAddr,4);
				if (ServiceEndpointChangedEvent)
					ServiceEndpointChangedEvent(serviceEndpoint);
			}
			break;
		case IpConfigSetMask:
			if (memcmp(localm[0].NetMask,data,4))
			{
				memcpy(localm[0].NetMask,data,4);
				mem.WriteMemory(IPCONFIG_ADDRESS+8,localm[0].NetMask,4);
				if (ServiceEndpointChangedEvent)
					ServiceEndpointChangedEvent(serviceEndpoint);
			}
			break;
		case IpConfigSetGateway:
			if (memcmp(localm[0].DefGW,data,4))
			{
				memcpy(localm[0].DefGW,data,4);
				mem.WriteMemory(IPCONFIG_ADDRESS+4,localm[0].DefGW,4);
				if (ServiceEndpointChangedEvent)
					ServiceEndpointChangedEvent(serviceEndpoint);
			}
			break;
		case IpConfigSetServiceEnpoint:
			if (memcmp(serviceEndpoint,data,6))
			{
				memcpy(serviceEndpoint,data,6);
				mem.WriteMemory(ENDPOINT_ADDRESS,serviceEndpoint,6);
				if (ServiceEndpointChangedEvent)
					ServiceEndpointChangedEvent(serviceEndpoint);
			}
			break;
		default:
			break;
	}
}

void NetworkConfig::GenerateMacAddress()
{
	std::uint32_t mac = GET_RANDOM_NUMBER;
	memcpy(eth0_config.MacAddr+3, &mac, 3);
	mem.WriteMemory(MAC_ADDRESS,eth0_config.MacAddr,6);
}

void NetworkConfig::CommandArrival(std::uint8_t command,std::uint8_t *parameters,std::size_t len)
{
	IpConfigItem item = IpConfigItem(command);
	uint8_t ack = 0;
	switch (item)
	{
		case IpConfigGetIpAddress:
		case IpConfigGetMask:
		case IpConfigGetGateway:
			comm.SendData(item, GetIpConfig(item), 4);
			break;
		case IpConfigGetServiceEnpoint:
			comm.SendData(IpConfigGetServiceEnpoint, GetIpConfig(IpConfigGetServiceEnpoint), 6);
			break;
		case IpConfigSetIpAddress:
		case IpConfigSetMask:
		case IpConfigSetGateway:
			if (len == 4)
				SetIpConfig(item, parameters);
			else
				ack = 1;
			comm.SendData(item, &ack, 1);
			break;
		case IpConfigSetServiceEnpoint:
			if (len == 6)
				SetIpConfig(IpConfigSetServiceEnpoint, parameters);
			else
				ack = 1;
			comm.SendData(item, &ack, 1);
			break;
		case IpConfigWhoAmI:
			ack = 0xaa;
			comm.SendData(IpConfigWhoAmI, NULL, 0);
			break;
//		case IpConfigDebugOn:
//			break;
//		case IpConfigDebugOff:
//			break;
	}
}

#pragma diag_warning 368

