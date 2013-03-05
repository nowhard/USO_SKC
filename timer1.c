#include "timer1.h"

//---------------------------------------------------------
void Timer1_Initialize(void) //initialize delay timer1
{
	TMOD &= 0xF; // 0000 1111
	TMOD |= 0x10; //16bit 

   	TL1 = 0xFF;
	TH1 = 0xE7;
	

	ET1=1;
	TR1=1;
	return;
}
