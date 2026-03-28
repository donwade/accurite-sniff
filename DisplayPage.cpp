#include <M5Unified.h>
#include <M5GFX.h>

#include "DisplayPage.hpp"

uint32_t vertPixel;
uint32_t lastMargin;

static char lclBuf[120];

void Home(int16_t marginLeft)
{
	if (marginLeft <= 0) // requesting neg h pos means top home cursor
	{
		vertPixel = 0;
		marginLeft = -marginLeft;
	}
	lastMargin = marginLeft;
	
	// first time using a page, you forgot to set left margin
	assert(vertPixel < 500);
	
	//Serial.printf("Home vertPixel = %d\n", vertPixel);
	
	M5.Lcd.setCursor(lastMargin, vertPixel);
	M5.Lcd.clear(TFT_BLACK);
}

uint16_t LineAdvance(void)
{
	// first time using a page, you forgot to set left margin
	assert(vertPixel < 500); 
	
	vertPixel += M5.Lcd.fontHeight(M5.Lcd.getFont());
	//Serial.printf("vertPixel = %d\n", vertPixel);
	M5.Lcd.setCursor(lastMargin, vertPixel);
	return vertPixel;
}

int lprintf(const String &foo)
{
	int ret = lprintf(foo.c_str());
	return ret;
}

int lprintf(const char* format, ... )
{
	va_list args;
    va_start(args, format); 

    int ret = vsnprintf(lclBuf, sizeof(lclBuf), format, args);
	M5.Lcd.print(lclBuf);
	
	LineAdvance();
	
	// POI Serial.printf("--->  %s\n", lclBuf); // print lcd prints to serial
    va_end(args); 
    return ret;
}

void paint(void) {
    M5.Lcd.display();
}

