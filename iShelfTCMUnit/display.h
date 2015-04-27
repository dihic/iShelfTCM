#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <cstdint>

using namespace std; 

#define MAXCHAR_PER_LINE 12
#define LINE_HEIGHT			 35
#define X_OFFSET				 10
#define RES_X						 400
#define RES_Y 					 240

namespace Display
{
	void ShowFormatString(const uint8_t *str, uint16_t size, uint16_t posy, float scale=1.0f);
	void ShowString(const uint8_t *str, uint16_t strlen, uint16_t posx, uint16_t posy, float scale=1.0f);
	void ShowString(uint32_t number, uint16_t posx, uint16_t posy, float scale=1.0f);
	void ShowAnsiString(const uint8_t *str, uint16_t strlen, uint16_t posx, uint16_t posy, float scale=1.0f);
	void ClearRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
	void SetColor(uint8_t r, uint8_t g, uint8_t b, uint8_t bk);
	void DisplayOnOff(bool on);
	void Brightness(uint8_t level);
}

#endif
