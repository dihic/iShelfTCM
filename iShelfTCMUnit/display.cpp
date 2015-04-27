#include "uart.h"
#include <cstdint>
#include <cstring>
#include "display.h"

using namespace std;

#define COMMAND_DRAW_STRING  		0xA1
#define COMMAND_DRAW_DIGIT 			0xA4
#define COMMAND_SET_COLOR 			0xA2
#define COMMAND_CLEAR_REGION 		0xA3
#define COMMAND_DISPLAY_ON_OFF 	0xA5
#define COMMAND_BRIGHTNESS 			0xA6

namespace Display
{
	uint8_t bytes[0x100];
	uint8_t bytesLen=0;
	
	void ShowFormatString(const uint8_t *str, uint16_t size, uint16_t posy, float scale)
	{
		uint16_t index = 0;
		uint16_t tail = 0;
		uint16_t len = 0;
		bool more = true;
		//bool newline = true;
		
		while (tail < size)
		{
			more = true;

			if (str[tail]==0 && str[tail+1]==0)
			{
				//ClearRegion(X_OFFSET, posy, RES_X-X_OFFSET, LINE_HEIGHT);
				if (len>0)
				{
					ShowString(str+index, len, X_OFFSET, posy, scale);
					index += (len<<1);
					len = 0;
					posy += LINE_HEIGHT;
				}
				index += 2;
				tail += 2;
				
				SetColor(255, 255, 255, 0);
				SetColor(0, 0, 0, 1);
				more = false;
				//newline = true;
				
			}
			else if (str[tail]==1 && str[tail+1]==0)
			{
				if (len>0)
				{
					ShowString(str+index, len, X_OFFSET, posy, scale);
					index += (len<<1);
					len = 0;
					posy += LINE_HEIGHT;
				}
				index += 8;
				
				SetColor(str[tail+2], str[tail+3], str[tail+4], 0);
				SetColor(str[tail+5], str[tail+6], str[tail+7], 1);
				more = false;
				//newline = true;
				tail += 8;
			}
			else
			{
				if (++len>=MAXCHAR_PER_LINE)
				{
					//ClearRegion(X_OFFSET, posy, RES_X-X_OFFSET, LINE_HEIGHT);
					ShowString(str+index, MAXCHAR_PER_LINE, X_OFFSET, posy, scale);
					more = false;
					index += (len<<1);
					len = 0;
					posy+=LINE_HEIGHT;
				}
				tail+=2;
			}
		}
		if (more)
		{
			//ClearRegion(X_OFFSET, posy, RES_X-X_OFFSET, LINE_HEIGHT);
			ShowString(str+index, len, X_OFFSET, posy, scale);
		}
		//Reset color
		SetColor(255, 255, 255, 0);
		SetColor(0, 0, 0, 1);
	}
	
	void ShowString(const uint8_t *str, uint16_t strlen, uint16_t posx, uint16_t posy, float scale)
	{
		uint16_t len = 10 + (strlen<<1);
		bytes[0] = COMMAND_DRAW_STRING;
		bytes[1] = strlen;
		bytes[2] = posx & 0xff;
		bytes[3] = posx >> 8;
		bytes[4] = posy & 0xff;
		bytes[5] = posy >> 8;
		memcpy(bytes+6,&scale,4);
		memcpy(bytes+10,str,strlen<<1);
		UARTSendSpecial(bytes, len);
	}
	
	void ShowAnsiString(const uint8_t *str, uint16_t strlen, uint16_t posx, uint16_t posy, float scale)
	{
		int i = 0;
		bytes[0] = COMMAND_DRAW_STRING;
		//bytes[1] = strlen;
		bytes[2] = posx & 0xff;
		bytes[3] = posx >> 8;
		bytes[4] = posy & 0xff;
		bytes[5] = posy >> 8;
		memcpy(bytes+6,&scale,4);
		while (i<strlen)
		{
			bytes[10+(i<<1)] = str[i];
			bytes[11+(i<<1)] = 0;
			++i;
		}
		bytes[1] = i;
		UARTSendSpecial(bytes, 10+(i<<1));
	}
	
	void ShowString(uint32_t number, uint16_t posx, uint16_t posy, float scale)
	{
		bytesLen = 14;
		bytes[0] = COMMAND_DRAW_DIGIT;
		bytes[1] = 0;
		bytes[2] = posx & 0xff;
		bytes[3] = posx >> 8;
		bytes[4] = posy & 0xff;
		bytes[5] = posy >> 8;
		memcpy(bytes+6,&scale,4);
		memcpy(bytes+10,&number,4);
		UARTSendSpecial(bytes,bytesLen);
	}

	void ClearRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
	{
		bytesLen=9;
		bytes[0] = COMMAND_CLEAR_REGION;
		bytes[1] = x & 0xff;
		bytes[2] = x >> 8;
		bytes[3] = y & 0xff;
		bytes[4] = y >> 8;
		bytes[5] = w & 0xff;
		bytes[6] = w >> 8;
		bytes[7] = h & 0xff;
		bytes[8] = h >> 8;
		UARTSendSpecial(bytes,bytesLen);
	}

	void SetColor(uint8_t r, uint8_t g, uint8_t b, uint8_t bk)
	{
		bytesLen=5;
		bytes[0] = COMMAND_SET_COLOR;
		bytes[1] = bk ? 1 : 0;
		bytes[2] = b;
		bytes[3] = g;
		bytes[4] = r;
		//buffer[5] = 0;
		UARTSendSpecial(bytes,bytesLen);
	}
	
	void DisplayOnOff(bool on)
	{
		bytesLen=2;
		bytes[0] = COMMAND_DISPLAY_ON_OFF;
		bytes[1] = on;
		UARTSendSpecial(bytes,bytesLen);
	}
	
	void Brightness(uint8_t level)
	{
		bytesLen=2;
		bytes[0] = COMMAND_BRIGHTNESS;
		bytes[1] = level;
		UARTSendSpecial(bytes,bytesLen);
	}
}
