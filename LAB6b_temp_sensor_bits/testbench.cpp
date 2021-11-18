// test at https://ideone.com/YvgSGJ

#include <iostream>
using namespace std;

int main() {
	float should_be[5] =             {	0,   -0.125,   -25.0 , -54.875, -55.000};   
	uint8_t temperature_bytes[2*5] = {0,0,0xFF,0xE0,0xE7,0x0,0xC9,0x20,0xC9,0x0};
	
	for(int i = 0; i<5; i++)
	{
		float ftemp = ((int16_t) ((uint16_t)temperature_bytes[2*i] << 8 | temperature_bytes[2*i+1]))/256.0;
		// float ftemp = temp / 256.0;
		cout << ftemp << ' ' << should_be[i]-ftemp <<endl;
	}

	return 0;
}