// RemoteControl.h

#ifndef _REMOTECONTROL_h
#define _REMOTECONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

typedef struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z; // I don't know what is this for but in the protocol there is a z axis for the mouse
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;
typedef	struct
{
	/*
		A 16-bit binary with each bit representing a key. When pressed, the cooresponding bit will be 1.
		Bit0-----W
		Bit1-----S
		Bit2-----A
		Bit3-----D
		Bit4-----Shift
		Bit5-----Ctrl
		Bit6-----Q
		Bit7-----E
		Bit8-----R
		Bit9-----F
		Bit10-----G
		Bit11-----Z
		Bit12-----X
		Bit13-----C
		Bit14-----V
		Bit15-----B
	*/
	uint16_t v;
}Key;

typedef struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

#endif

