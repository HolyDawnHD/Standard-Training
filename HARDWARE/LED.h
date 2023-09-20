#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED_G PCout(2)
#define LED_R PCout(3)

void LED_Init(void);

#endif

