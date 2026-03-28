#pragma once
#include <cstdint>

#include "M5Unified.h"
#include "M5GFX.h"
int lprintf(const char* format, ... );
int lprintf(const String &foo);
int lprintf(int16_t leftMargin, const String &foo);
void Home(int16_t marginLeft = 0);
void paint(void);

