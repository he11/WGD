#ifndef _HANGUL_H_
#define _HANGUL_H_

#include "ssd1306.h"

// Variable
extern byte HANFontImage[32];

// Function
extern void matrixPrint(int XPOS, int YPOS, char* pChar);
#endif
