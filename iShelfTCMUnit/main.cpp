#include <LPC11xx.h>
#include <cstring>
#include <cmath>
#include "gpio.h"
#include "uart.h"
#include "timer32.h"
#include "ssp.h"
#include "trf796x.h"
#include "iso15693.h"
#include "SensorArray.h"
#include "canonchip.h"
#include "fram.h"
#include "Display.h"
#include "CardInfo.h"
#include "delay.h"

using namespace std;

#define EINT_PIN PORT1,2

#define MAX_ZERO_DRIFT  2
#define ZERO_DRIFT_COEFF 0.2f
#define INTRUSION_EXTENSION_TIME 500   //500ms

#define MEM_BUFSIZE 0x200

#define NVDATA_ADDR 0x10
#define NVINFO_ADDR 0x80

#define NVDATA_BASE 0x30

#define SYNC_SENSOR     0x0101
#define SYNC_GOTCHA			0x0180
#define SYNC_DATA				0x0100
#define SYNC_LIVE				0x01ff

#define OP_SET_ZERO 	0xfe
#define OP_RAMPS			0x8000
#define OP_AUTO_RAMPS	0x8007
#define OP_UNIT     	0x8001
#define OP_CAL_UNIT   0x8002
#define OP_CONFIG     0x8003
#define OP_RAWDATA		0x8006
#define OP_NOTICE     0x9000

//#define A_POS_X		5
//#define A_POS_Y		160
//#define B_POS_X 	205
//#define B_POS_Y		160
//#define DIGIT_W		195
//#define DIGIT_H		80
//#define DIGIT_SCALE 2.0f

#define TEXT_POS_Y	10
#define TEXT_POS_X	5
#define INDICATOR_POS_Y	180 

const static uint8_t NoticeStr[] = { 0x05, 0x2a, 0x67, 0xd1, 0x7e, 0x9a, 0x5b, 0x6f, 0x83, 0xee, 0x7b }; // Unbinded notice string
#define NOTICE_X	125
#define NOTICE_Y	95

#define NAME_SCALE 1.5f

__align(16) volatile uint8_t MemBuffer[MEM_BUFSIZE];

CardInfo *cardInfo;

SensorArray sensorArray;

int32_t *pZero[4];
float *pRamp[4];
float *pUnit;
float *pDeviation;
float *pCalWeight;
uint16_t *pSensorType;
uint8_t *pSensorDisable;
int32_t *pCurrentValue[4];

uint8_t *pRfidCardType;

float *pWeightRatio[4];

uint8_t buf[300];
volatile uint8_t i_reg = 0x01;							// interrupt register
volatile uint8_t irq_flag = 0x00;
volatile uint8_t rx_error_flag = 0x00;
volatile uint8_t rxtx_state = 1;							// used for transmit recieve byte count
volatile uint8_t host_control_flag = 0;
volatile uint8_t stand_alone_flag = 1;
extern uint8_t UID[8];

extern volatile uint32_t UARTStatus;
extern volatile uint8_t UARTBuffer[UART_BUFSIZE];
extern volatile uint32_t UARTCount;

uint8_t AckBuffer[UART_BUFSIZE];
uint16_t AckLength;

struct CanResponse
{
	uint16_t sourceId;
	CAN_ODENTRY response;
	uint8_t result;
};

CanResponse res;

volatile bool syncTriggered = false;
volatile bool responseTriggered = false;
volatile CAN_ODENTRY syncEntry;

float CurrentNetWeight=0;
float BackupWeight=0;
int16_t BackupValue[4];

volatile bool Connected = true;		// host initialize connection, so set true when powerup 
volatile bool Registered = false;		// Registered by host
volatile bool ForceSync = false;
volatile bool Gotcha = true;
volatile bool DataLock = false;
volatile bool RfidTimeup = true;	// for power saving


#define ADDR_ZERO  					0x00
#define ADDR_RAMP  					0x10
//#define ADDR_UNIT						0x20
//#define ADDR_DEVIATION			0x24
#define ADDR_CAL_WEIGHT			0x28
#define ADDR_SENSOR_TYPE		0x2C
#define ADDR_SENSOR_DISABLE	0x2E
#define ADDR_CURRENT				0x30
//#define ADDR_Q							0x40
//#define ADDR_Q_BACKUP				0x44
//#define ADDR_Q_PRES					0x48
//#define ADDR_Q_GONE					0x4C
#define ADDR_WEIGHT_RATIO   0x50
//#define ADDR_MED_GUID				0x60
//#define ADDR_UNIT						0x70
//#define ADDR_DEVIATION			0x74
//#define ADDR_PRES_ID_LEN		0x78
//#define ADDR_PRES_ID_NAME		0x79

void Setup()
{
	uint8_t id,i;
 	id = (LPC_GPIO[PORT2]->MASKED_ACCESS[0x1C0])>>6;
	id |= (LPC_GPIO[PORT2]->MASKED_ACCESS[0xC00])>>7;
	if (GPIOGetValue(PORT1,10))
		id|=0x20;
	if (GPIOGetValue(PORT1,11))
		id|=0x40;
	NodeId = 0x100 | id;
	
	memset((void *)MemBuffer,0,MEM_BUFSIZE);
	MemBuffer[0]=0xEE;
	
	pRfidCardType = (uint8_t *)(MemBuffer+1);
	
	for(i=0;i<4;++i)
	{
		pZero[i] = 				 (int32_t *)(MemBuffer + NVDATA_BASE + ADDR_ZERO + i*4);
		pRamp[i] = 				 	  (float *)(MemBuffer + NVDATA_BASE + ADDR_RAMP + i*4);
		pCurrentValue[i] = (int32_t *)(MemBuffer + NVDATA_BASE + ADDR_CURRENT + i*4);
		pWeightRatio[i]  = 		(float *)(MemBuffer + NVDATA_BASE + ADDR_WEIGHT_RATIO + i*4);
	}
	pCalWeight =         (float *)(MemBuffer + NVDATA_BASE + ADDR_CAL_WEIGHT);
	pSensorType =     (uint16_t *)(MemBuffer + NVDATA_BASE + ADDR_SENSOR_TYPE);
	pSensorDisable =   (uint8_t *)(MemBuffer + NVDATA_BASE + ADDR_SENSOR_DISABLE);
}

uint8_t syncBuf[0x30];

void PrepareSyncData()
{
	syncEntry.index=SYNC_DATA;
	syncEntry.subindex=0;
	syncEntry.val=syncBuf;
	
	uint8_t *presId;
//	if (Connected && !DataLock)
//	{
		syncBuf[0] = *pRfidCardType; 		//Card Type
		memcpy(syncBuf+1, (void *)(MemBuffer+0x20), 8);	  	//Card Id
		if (*pRfidCardType == 2)
		{
			presId = cardInfo->GetPresId(syncBuf[9]);
			memcpy(syncBuf+10, presId, syncBuf[9]);
			syncEntry.entrytype_len = 10+syncBuf[9];
		}
		else
			syncEntry.entrytype_len = 9;
		syncTriggered = true;
//	}
}


#define RFID_TIME_COUNT    10
#define RFID_TIME_INTERVAL 50

extern "C" {

//volatile uint8_t lastState=0;	//0:intrusion 1:stable
	
void TIMER32_0_IRQHandler()
{
	static uint32_t counter1=0;
	static uint32_t counter2=0;
	static bool atOnce = false;
	
	if ( LPC_TMR32B0->IR & 0x01 )
  {    
		LPC_TMR32B0->IR = 1;			/* clear interrupt flag */
		
		if (!RfidTimeup)
		{
			if (counter2++>=RFID_TIME_INTERVAL)
			{
				RfidTimeup =true;
				counter2 = 0;
			}				
		}
		
		if (atOnce && !DataLock)
		{
			atOnce = false;
			PrepareSyncData();
			return;
		}
		
		if (counter1++>=HeartbeatInterval)
		{
			counter1=0;
			if (Connected)
			{
				if (Registered && !Gotcha)
				{
					if (DataLock)
						atOnce = true;
					else
						PrepareSyncData();
				}
			}
			else
				CANEXHeartbeat(STATE_OPERATIONAL);
		}
	}
}

//void CalibrateSensors(uint8_t flags)
//{
//	for(uint8_t i=0;i<4;++i)
//	{
//		if ((flags & (1<<i)) == 0)
//			continue;
//		*pRamp[i]=(*pCurrentValue[i]-*pZero[i])/(*pCalWeight);
//	}
//	FRAM::WriteMemory(NVDATA_ADDR + ADDR_RAMP,
//					(uint8_t *)(pRamp[0]),4*sizeof(float));
//}

//volatile uint8_t Updating=0;

//void UpdateWeight()
//{
//	float weight=0;
//	float weightArray[4];
//	float zeroWeight=0;	// , zeroWeightTemp;
//	
//	static uint8_t extend = 0;
//	
//	Updating=1;
//	
//	bool fail = false;
//	
//	for(int i=0;i<4;++i)
//	{
//		weightArray[i] = 0;
//		
//		//Skip disabled sensors
//		if (*pSensorDisable & (1<<i))
//		{
//			*pCurrentValue[i] = 0;
//			continue;
//		}
//	
//		//if (sensorArray.pSensors[i]->CurrentData)
//			*pCurrentValue[i] = sensorArray.pSensors[i]->CurrentData;
//		
//		//Calculation...
//		//diff=*pCurrentValue[i] -*pZero[i];
//		//if (*pRamp[i]>0.0f && *pRamp[i]<10000.0f)
//		if (abs(*pRamp[i])>0.000001f)
//		{
//			weightArray[i] = (*pZero[i])/(*pRamp[i]);
//			zeroWeight += weightArray[i];
//			weight += (*pCurrentValue[i])/(*pRamp[i]);
//		}
//		else
//			fail = true;
//	}
//	
//	if (fail)
//	{
//		Updating = 0;
//		return;
//	}
//	
////	for(int i=0;i<4;++i)
////		*pWeightRatio[i] = (zeroWeight!=0)?weightArray[i]/zeroWeight:0;
//	
//	//Determine whether intrusion happen or not
//	//zeroWeightTemp = weight - BackupWeight;
//	//zeroWeightTemp = weight - (BackupQuantity * (*pUnit) + zeroWeight);
//	//lastState = abs(zeroWeightTemp) < (*pDeviation) * (*pUnit);
//	BackupWeight = weight;
//	
//	//Intrusion Detected, calculate quantity
//	if (lastState==0 || extend)
//	{
//		CurrentNetWeight = weight - zeroWeight;// - *pPackageWeight;
//		
//		float q = (*pUnit==0.0f)? 0 : CurrentNetWeight/(*pUnit);
//		if (q<0)
//		{
//			//*pQuantity = 0;
//			q = 0;
//		}
//		else
//		{
//			//*pQuantity = (uint32_t)(q+0.5f);
//			//if ((*pPresAQuantity>0 || *pPresBQuantity>0) && *pBackupQuantity == 0xFFFFFFFFu)
//			//{
//			//	*pBackupQuantity=*pQuantity;
//			//}				
//			//GoneQuantity available when backup quantity exists
//			//*pGoneQuantity = (*pBackupQuantity>0)?(*pBackupQuantity)-(*pQuantity):0;
//		}
//		//BackupQuantity = *pQuantity;
//		if (extend == 0)
//			extend = 1;
//		else
//			if (++extend > 5)
//				extend = 0;
//	}
//	else //Drift Reduction
//	{
//		for(int i=0;i<4;++i)
//		{
//			//Skip disabled sensors
//			if (*pSensorDisable & (1<<i))
//				*pCurrentValue[i]=0;
//			else
//				//(*pZero[i]) += zeroWeightTemp * (*pWeightRatio[i]) * (*pRamp[i]);
//				(*pZero[i]) += (*pCurrentValue[i]) - BackupValue[i];
//		}
//		FRAM::WriteMemory(NVDATA_ADDR + ADDR_ZERO,(uint8_t *)(pZero[0]),16);
//	}
//	for(int i=0;i<4;++i)
//		BackupValue[i] = (*pCurrentValue[i]);
//	Updating=0;
//}

//void TIMER32_1_IRQHandler()
//{
//	static int counter=0;

//	if ( LPC_TMR32B1->IR & 0x01 )
//  {    
//		LPC_TMR32B1->IR = 1;			/* clear interrupt flag */
//		sensorArray.Refresh();
//	
//		if (++counter >= 100)
//		{
//			counter = 0;
//			UpdateWeight(); 
//		}
//	}
//}

void DrawNoticeBar();

void CanexReceived(uint16_t sourceId, CAN_ODENTRY *entry)
{
	CAN_ODENTRY *response=const_cast<CAN_ODENTRY *>(&(res.response));

	res.sourceId = sourceId;
	res.result=0xff;
	
	response->val = const_cast<uint8_t *>(&(res.result));
	response->index = entry->index;
	response->subindex = entry->subindex;
	response->entrytype_len = 1;

	switch (entry->index)
	{
		case 0:	//system config
			break;
//		case OP_SET_ZERO:
//			disable_timer32(1);
//			while (Updating);
//			for(i=0;i<4;++i)
//			{
//				if (*pSensorDisable & (1<<i))
//					continue;
//				*pZero[i] = sensorArray.pSensors[i]->CurrentData;
//			}
//			//*pQuantity=0;
//			CurrentNetWeight=0;
//			FRAM::WriteMemory(NVDATA_ADDR + ADDR_ZERO,(uint8_t *)(pZero[0]),0x10);
//			reset_timer32(1);
//			enable_timer32(1);
//			*(response->val)=0;
//			break;
//		case OP_RAWDATA:
//			response->entrytype_len=0x20;
//			response->val=buf;
//			memcpy(buf,pZero[0],0x10);
//			memcpy(buf+0x10,pCurrentValue[0],0x10);
//			break;
//		case OP_AUTO_RAMPS:
//			CalibrateSensors(entry->val[0]);
//			*(response->val)=0;
//			break;
//		case OP_RAMPS: //R/W coeff
//			if (entry->subindex==0)
//			{
//				response->entrytype_len=4*sizeof(float);
//				response->val=(uint8_t *)(pRamp[0]);
//			}
//			else
//			{
//				if (entry->val[0]>=4)
//					break;
//				memcpy((uint8_t *)(pRamp[entry->val[0]]),entry->val+1,sizeof(float));
//				FRAM::WriteMemory(NVDATA_ADDR + ADDR_RAMP + entry->val[0]*sizeof(float),
//					(uint8_t *)(pRamp[entry->val[0]]),sizeof(float));
//				*(response->val)=0;
//			}
//			break;
//		case OP_CAL_UNIT:
//			if (entry->subindex==0)
//			{
//				response->entrytype_len=sizeof(float);
//				response->val=(uint8_t *)pCalWeight;
//			}
//			else
//			{
//				if (entry->entrytype_len<sizeof(float))
//					break;
//				memcpy(pCalWeight,entry->val,sizeof(float));
//				FRAM::WriteMemory(NVDATA_ADDR + ADDR_CAL_WEIGHT,(uint8_t *)pCalWeight,sizeof(float));
//				*(response->val)=0;
//			}
//			break;
//		case OP_CONFIG:
//			if (entry->subindex==0)
//			{
//				response->entrytype_len = 3;
//				response->val=(uint8_t *)pSensorType;
//			}
//			else
//			{
//				if (entry->entrytype_len<3)
//					break;
//				*pSensorType=entry->val[0]|(entry->val[1]<<8);
//				*pSensorDisable=entry->val[2];
//				FRAM::WriteMemory(NVDATA_ADDR + ADDR_SENSOR_TYPE,(uint8_t *)pSensorType,3);
//				*(response->val)=0;
//			}
//			break;
		case OP_NOTICE:
			if (entry->subindex!=0)
			{
				cardInfo->SetNotice(entry->val[0]);
				DrawNoticeBar();
				*(response->val)=0;
			}
			break;
	}
	//CANEXResponse(sourceId,response);
	responseTriggered = true;
}

void CanexSyncTrigger(uint16_t index, uint8_t mode)
{	
	if (mode!=0)
		return;
	if (syncTriggered)
		return;
	
	syncEntry.index=index;
	syncEntry.subindex=0;
	syncEntry.val=syncBuf;
	
	switch(index)
	{
//		case SYNC_SENSOR:
//			if (Connected)
//			{
//				syncEntry.entrytype_len=0x20;
//				memcpy(syncBuf,pZero[0],0x10);
//				memcpy(syncBuf+0x10,pCurrentValue[0],0x10);
//				syncTriggered = true;
//			}
//			break;
		case SYNC_GOTCHA:
			Gotcha = true;
			break;
		case SYNC_DATA:
			ForceSync = true;
			break;
		case SYNC_LIVE:
			Connected=!Connected;
			if (Connected)
				Registered = true;
			break;
	}
}

}

void AckReciever();

void UARTProcessor()
{
	static uint8_t phase=0;
	static uint8_t sumcheck=0;
	static uint16_t readIndex=0;
	static uint32_t UARTPostion=0;
	
	while (UARTPostion!=UARTCount)
	{
		uint8_t byte=UARTBuffer[UARTPostion++];
		if (UARTPostion>=UART_BUFSIZE)
			UARTPostion=0;
		switch (phase)
		{
			case 0:
				if (byte==REV_HEADER)
				{
					phase=1;
					readIndex=0;
					AckLength=0;
					sumcheck=REV_HEADER;
				}
				break;
			case 1:
				sumcheck+=byte;
				AckLength|=byte<<(readIndex*8);
				readIndex++;
				if (readIndex>=2)
				{
					phase=2;
					readIndex=0;
				}				
				break;
			case 2:
				sumcheck+=byte;
				AckBuffer[readIndex++]=byte;
				if (readIndex>=AckLength)
					phase=3;
				break;
			case 3:
				if (byte==sumcheck)
					AckReciever();
				phase=0;
				break;
		}
	}
}

void DrawNoticeBar()
{
	const uint8_t *bc = cardInfo->GetNoticeColor();
	Display::SetColor(bc[0], bc[1], bc[2], 1);
	Display::ClearRegion(0, INDICATOR_POS_Y, RES_X, RES_Y - INDICATOR_POS_Y);
	Display::SetColor(0, 0, 0, 1);
}

void RefreshDisplay()
{
	uint8_t *p;
	uint8_t len;
	const uint8_t *color;
	switch (*pRfidCardType)
	{
		case 0:
			Display::DisplayOnOff(false);
			Display::ClearRegion(0, 0, RES_X, RES_Y);
			break;
		case 1:
			Display::ClearRegion(0, 0, RES_X, RES_Y);
			Display::DisplayOnOff(true);
			Display::ShowString((uint8_t *)(NoticeStr+1), NoticeStr[0], NOTICE_X, NOTICE_Y);
			break;
		case 2:
			Display::ClearRegion(0, 0, RES_X, RES_Y);
			Display::DisplayOnOff(true);
			color = cardInfo->GetPriorityColor();
			Display::SetColor(color[0], color[1], color[2], 0);
			p = cardInfo->GetQueueId(len);
			Display::ShowString(p, len, TEXT_POS_X, TEXT_POS_Y);
			p = cardInfo->GetPresId(len);
			Display::ShowAnsiString(p, len, TEXT_POS_X, TEXT_POS_Y + LINE_HEIGHT);
			p = cardInfo->GetName(len);
			Display::ShowString(p, len, TEXT_POS_X, TEXT_POS_Y + LINE_HEIGHT*2, NAME_SCALE);
			Display::SetColor(0xff, 0xff, 0xff, 0);
			DrawNoticeBar();
			break;
		default:
			break;
	}
}

void AckReciever()
{
	if (AckLength>0 && AckBuffer[0] == 0xff)
		RefreshDisplay();
}

bool UpdateRfid()
{	
	static uint8_t UIDlast[8];
	static uint32_t lostCount = 0;
	
	bool result = false;
	uint8_t suc = 0;
	uint8_t failCount;
	
	Trf796xTurnRfOn();
	DELAY(2000);
	if (Iso15693FindTag())
	{
		lostCount = 0;
		if (memcmp(UID, UIDlast, 8) != 0)
		{
			result = true;
			memcpy(UIDlast, UID, 8);
			memcpy((void *)(MemBuffer+0x20), UID, 8);
			
			failCount = 0;
			do
			{
				suc = Iso15693ReadSingleBlockWithAddress(0, UID, cardInfo->GetHeader());
				DELAY(1000);
			} while (suc && ++failCount<10);
			
			if (cardInfo->IsValid())
			{
				*pRfidCardType = 0x02;
				memcpy(cardInfo->UID, UID, 8);
				failCount = 0;
				do {
					suc = Iso15693ReadMultipleBlockWithAddress(1, 27, UID, cardInfo->GetData());
					DELAY(1000);
				} while (suc && ++failCount<10);
				if (failCount>=10)
				{
					*pRfidCardType = 0x00;
					memset(UIDlast, 0, 8);
				}
			}
			else
			{
				if (failCount>=10)
				{
					*pRfidCardType = 0x00;
					memset(UIDlast, 0, 8);
				}
				else
				{
					*pRfidCardType = 0x01;
					memcpy(cardInfo->UID, UID, 8);
				}
			}
			RefreshDisplay();
		}
		if (*pRfidCardType == 0x02 && cardInfo->NeedUpdate())
		{
			do {
				suc = Iso15693WriteSingleBlock(1, cardInfo->UID, cardInfo->GetData());
				DELAY(1000);
			} while (suc);
		}
	}
	else
	{
		if (++lostCount>RFID_TIME_COUNT)
		{
			lostCount = RFID_TIME_COUNT;
			if (*pRfidCardType!=0)
			{
				result = true;
				*pRfidCardType = 0x00;
				memset(UIDlast, 0, 8);
				RefreshDisplay();
			}
		}
	}
	Trf796xTurnRfOff();
	DELAY(2000);
	return result;
}

int main()
{
	SystemCoreClockUpdate();
	GPIOInit();
	
	cardInfo = new CardInfo; 
	
	Setup();
	sensorArray.Setup();
	
	UARTInit(230400);
	CANInit(500);
	CANEXReceiverEvent = CanexReceived;
	CANTEXTriggerSyncEvent = CanexSyncTrigger;
	
	DELAY(100000); // wait 100ms for voltage stable
	uint8_t firstUse=FRAM::Init();
	
	//firstUse=1;
	if (firstUse)
	{
		FRAM::WriteMemory(NVDATA_ADDR,(uint8_t *)(MemBuffer+NVDATA_BASE),0x30);
	}
	else
	{
		FRAM::ReadMemory(NVDATA_ADDR,(uint8_t *)(MemBuffer+NVDATA_BASE),0x30);
	}
	
	Trf796xCommunicationSetup();
	Trf796xInitialSettings();
	
	//UpdateWeight();
	
	init_timer32(0,TIME_INTERVAL(1000));	//	1000Hz
	//init_timer32(1,TIME_INTERVAL(100));		//	100Hz
	DELAY(10);
	enable_timer32(0);
	//enable_timer32(1);
	
	//DELAY(1000000); 	//Await 1sec
	
	while(1)
	{
		UARTProcessor();
		
		if (RfidTimeup)
		{
			RfidTimeup = false;
			DataLock = true;
			bool updated = UpdateRfid();
			if (Registered && (updated || ForceSync))
			{
				ForceSync = false;
				Gotcha = false;
			}
			DataLock = false;
		}
		
		if (responseTriggered)
		{
			CANEXResponse(res.sourceId, const_cast<CAN_ODENTRY *>(&(res.response)));
			responseTriggered = false;
		}
		
		if (syncTriggered)
		{
			syncTriggered = false;
			CANEXBroadcast(const_cast<CAN_ODENTRY *>(&syncEntry));
		}
	}
}
