#include "crc16.h"
/* A fairly simple Modbus compliant 16 Bit CRC algorithm.
*  Returns 1 if the crc check is positive, returns 0 if it fails.
*  Appends
*/
uint16_t crc16(volatile uint8_t *ptrToArray,uint8_t inputSize) //A standard CRC algorithm
{
	uint16_t out=0xffff;
	uint16_t carry,l;
	unsigned char n;
	inputSize++;
	for ( l=0; l<inputSize; l++) {
		out ^= ptrToArray[l];
		for (n = 0; n < 8; n++) {
			carry = out & 1;
			out >>= 1;
			if (carry) out ^= 0xA001;
		}
	}
	//out=0x1234;
	if ((ptrToArray[inputSize]==out%256) && (ptrToArray[inputSize+1]==out/256)) //check
	{
		return 1;
	} else { 
		ptrToArray[inputSize]=out%256; //append Lo
		ptrToArray[inputSize+1]=out/256; //append Hi
		return 0;	
	}
}

uint16_t makecrc16(char *ptrToArray,uint8_t inputSize) //A standard CRC algorithm
{
	uint16_t out=0xffff;
	uint16_t carry,l;
	unsigned char n;
	inputSize++;
	for ( l=0; l<inputSize; l++) {
		out ^= ptrToArray[l];
		for (n = 0; n < 8; n++) {
			carry = out & 1;
			out >>= 1;
			if (carry) out ^= 0xA001;
		}
	}
	//out=0x1234;
	if ((ptrToArray[inputSize]==out%256) && (ptrToArray[inputSize+1]==out/256)) //check
	{
		return 1;
	} else { 
		ptrToArray[inputSize]=out%256; //append Lo
		ptrToArray[inputSize+1]=out/256; //append Hi
		return 0;	
	}
}
