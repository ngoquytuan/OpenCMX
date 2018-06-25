//http://www.simplymodbus.ca/FC03.htm
#include "crc16.h"
#include "uart.h"
#include "modbus_define.h"
#include "delay.h"
const uint8_t SlaveID    = 0x11;//The Slave Address//ID (11 hex = address17 ),0 -> 255
const uint8_t funcCode   = 0x03;// The Function Code 3 (read Analog Output Holding Registers)
const uint8_t sizeOfData = 0x08;//The number of data bytes to follow (3 registers x 2 bytes each = 6 bytes), 1 -> 125
typedef struct  {
    uint8_t slaveID;
    uint8_t func;
	  uint8_t size;
	  uint8_t data[sizeOfData+1];
	  //uint16_t crc16;
} modbusSlave;

modbusSlave resfc03;

char     sendData[20];

/*************************************************************************************/
	 int8 coils = 2;
   int8 inputs = 6;
   int16 hold_regs[] = {0x8800,0x7700,0x6600,0x5500,0x4400,0x3300,0x2200,0x1100};
   int16 input_regs[] = {0x1100,0x2200,0x3300,0x4400,0x5500,0x6600,0x7700,0x8800};
   int16 event_count = 0;
	 uint8_t modbus_serial_new=0;
	 uint8_t MODBUS = 1;// on modbus mode
/*************************************************************************************/	 

void updateModbusData( void)
{
	char i;
	
	resfc03.data[0] =0xAE;
	resfc03.data[1] =0x41;
	resfc03.data[2] =0x56;
	resfc03.data[3] =0x52;
	resfc03.data[4] =0x43;
	resfc03.data[5] =0x40;
	resfc03.data[6] =0;
	resfc03.data[7] =0x01;
	resfc03.data[sizeOfData] = 0;
	resfc03.func    = funcCode;
	resfc03.size    = sizeOfData;
	resfc03.slaveID = SlaveID;
	
	
	sendData[0] = resfc03.slaveID;
	sendData[1] = resfc03.func;
	sendData[2] = resfc03.size;/*
	sendData[3] = resfc03.data[0]>>8;
	sendData[4] = resfc03.data[0];
	sendData[5] = resfc03.data[1]>>8;
	sendData[6] = resfc03.data[1];
	sendData[7] = resfc03.data[2]>>8;
	sendData[8] = resfc03.data[2];*/
	for(i=3;i<(3+sizeOfData);i++)
	{
	 sendData[i] = resfc03.data[i-3];
	}
	
	//sprintf(sendData,"%c%c%c%s\r\n",SlaveID,funcCode,sizeOfData,resfc03.data);
	makecrc16(sendData,sizeOfData+2);
	//printf("%s\r\n",sendData);
	u2Transmit(sendData,sizeOfData+5);
}

//void updateModbusDataTest(void)
//{
//  resfc03.slaveID = SlaveID;
//	resfc03.func    = funcCode;
//	resfc03.size    = sizeOfData;
//	resfc03.data[0] = 1;
//	resfc03.data[1] = 0xfe;
//	resfc03.data[2] = 0xfff;
//	u2Transmit(&resfc03,9);
//	//makecrc16(resfc03,9);
//}

//---------------------------------------------------------------MODBUS_APP_LAYER_C
// Purpose:    Initialize RS485 communication. Call this before
//             using any other RS485 functions.
// Inputs:     None
// Outputs:    None
void modbus_init()
{
   //Turn on uart
}

// Purpose:    Get a message from the RS485 bus and store it in a buffer
// Inputs:     None
// Outputs:    TRUE if a message was received
//             FALSE if no message is available
// Note:       Data will be filled in at the modbus_rx struct:
int1 modbus_kbhit()
{
   modbus_check_timeout();

   if(!modbus_serial_new) return FALSE;
   else if(modbus_rx.func & 0x80)           //did we receive an error?
   {
      modbus_rx.error = modbus_rx.data[0];  //if so grab the error and return true
      modbus_rx.len = 1;
   }
   modbus_serial_new=FALSE;
	 
   return TRUE;
}

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       func               function to respond to
           exception   error              exception response to send
Output:    void
*/
void modbus_exception_rsp(  uint8_t address,   uint16_t func, exception error)
{
   printf("modbus_exception_rsp\r\n");
	 modbus_serial_send_start(address, func|0x80);
   modbus_serial_putc(error);
   modbus_serial_send_stop();
}
/*
read_discrete_input_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_discrete_input_rsp(  uint8_t address,   uint8_t byte_count,
                                      uint8_t *input_data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_DISCRETE_INPUT);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*input_data);
      input_data++;
   }

   modbus_serial_send_stop();
}
/*
read_holding_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      reg_data           Pointer to an array of data to send
Output:    void
*/
void modbus_read_holding_registers_rsp(  uint8_t address,   uint8_t byte_count,
                                          uint16_t *reg_data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=2)
   {
      modbus_serial_putc(((*reg_data)>>8)&0xff);
      modbus_serial_putc((*reg_data)&0xff);
      reg_data++;
   }

   modbus_serial_send_stop();
}
/*
read_input_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_input_registers_rsp(  uint8_t address,   uint8_t byte_count,
                                          uint16_t *input_data)																					
{
   int8 i;

   modbus_serial_send_start(address, FUNC_READ_INPUT_REGISTERS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=2)
   {
      modbus_serial_putc(((*input_data)>>8)&0xff);
      modbus_serial_putc((*input_data)&0xff);
      input_data++;
   }

   modbus_serial_send_stop();
}

/*
write_single_coil_rsp
Input:     int8       address            Slave Address
           int16      output_address     Echo of output address received
           int16      output_value       Echo of output value received
Output:    void
*/
void modbus_write_single_coil_rsp(  uint8_t address,   uint16_t output_address,
                                      uint16_t output_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_COIL);

   modbus_serial_putc((output_address>>8)&0xff);
   modbus_serial_putc(output_address&0xff);

   modbus_serial_putc((output_value>>8)&0xff);
   modbus_serial_putc(output_value&0xff);

   modbus_serial_send_stop();
}
/*
read_exception_status_rsp
Input:     int8       address            Slave Address
Output:    void
*/
void modbus_read_exception_status_rsp(  uint8_t address,   uint8_t data)
{
   modbus_serial_send_start(address, FUNC_READ_EXCEPTION_STATUS);
   modbus_serial_send_stop();
}

/*
write_single_register_rsp
Input:     int8       address            Slave Address
           int16      reg_address        Echo of register address received
           int16      reg_value          Echo of register value received
Output:    void
*/
void modbus_write_single_register_rsp(  uint8_t address,   uint16_t reg_address,
                                          uint16_t reg_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_REGISTER);

   modbus_serial_putc((reg_address>>8)&0xff);
   modbus_serial_putc(reg_address&0xff);

   modbus_serial_putc((reg_value>>8)&0xff);
   modbus_serial_putc(reg_value&0xff);

   modbus_serial_send_stop();
}

/*
write_multiple_coils_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of coils written to
Output:    void
*/
void modbus_write_multiple_coils_rsp(  uint8_t address,   uint16_t start_address,
                                          uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_COILS);

   modbus_serial_putc((start_address>>8)&0xff);
   modbus_serial_putc(start_address&0xff);

   modbus_serial_putc((quantity>>8)&0xff);
   modbus_serial_putc(quantity&0xff);

   modbus_serial_send_stop();
}

/*
write_multiple_registers_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of registers written to
Output:    void
*/
void modbus_write_multiple_registers_rsp(  uint8_t address,   uint16_t start_address,
                                              uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc((start_address>>8)&0xff);
   modbus_serial_putc(start_address&0xff);

   modbus_serial_putc((quantity>>8)&0xff);
   modbus_serial_putc(quantity&0xff);

   modbus_serial_send_stop();
}
//---------------------------------------------------------------MODBUS_APP_LAYER_C

//--------------------------------------------------------------------phy layer RTU
#if (MODBUS_TYPE == MODBUS_TYPE_MASTER)
 uint32_t modbus_serial_wait=MODBUS_SERIAL_TIMEOUT;
#endif

/*Stages of MODBUS reception.  Used to keep our ISR fast enough.*/
enum {MODBUS_GETADDY=0, MODBUS_GETFUNC=1, MODBUS_GETDATA=2} modbus_serial_state = 0;


/*Global value holding our current CRC value.*/
union
{
    uint8_t b[2];
    uint16_t d;
} modbus_serial_crc;


/* Table of CRC values for high–order byte */
const unsigned char modbus_auchCRCHi[] = {
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40
};

/* Table of CRC values for low–order byte */
const unsigned char modbus_auchCRCLo[] = {
0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,
0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,
0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,
0x37,0xF5,0x35,0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,
0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,
0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,
0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,0x88,
0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,
0x40
};

#define MODBUS_GETDATA_TIMEOUT 40
uint32_t modbus_sticks;
#define WAIT_FOR_HW_BUFFER() printf("WAIT_FOR_HW_BUFFER\r\n")
uint32_t get_ticks(void)
{
	return modbus_sticks;
}
/* status of between byte timeout */
int1 modbus_timeout_enabled = _false;

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None
void RCV_ON(void)
{
	printf("RCV_ON\r\n");
}
#define RCV_OFF() {printf("RCV_OFF\r\n");}//Turn off ngat???

void set_ticks(uint32_t x)
{
	modbus_sticks = x;
}
// Purpose:    Send a message over the RS485 bus
// Inputs:     1) The destination address
//             2) The number of bytes of data to send
//             3) A pointer to the data to send
//             4) The length of the data
// Outputs:    TRUE if successful
//             FALSE if failed
// Note:       Format:  source | destination | data-length | data | checksum
void modbus_serial_send_start( uint8_t to,  uint8_t func)
{
   modbus_serial_crc.d=0xFFFF;
   modbus_serial_new=FALSE;

   //RCV_OFF();

   delay_us(3500000/MODBUS_SERIAL_BAUD); //3.5 character delay

   modbus_serial_putc(to);
   modbus_serial_putc(func);
}

// Purpose:    Ends a message over the RS485 Bus
// Inputs:     Character
// Outputs:    None
void modbus_serial_send_stop()
{
   uint8_t crc_low, crc_high;

   crc_high=modbus_serial_crc.b[1];
   crc_low=modbus_serial_crc.b[0];

   modbus_serial_putc(crc_high);
   modbus_serial_putc(crc_low);


   delay_us(3500000/MODBUS_SERIAL_BAUD); //3.5 character delay

   //RCV_ON();


   modbus_serial_crc.d=0xFFFF;
}

// Purpose:    Check if we have timed out waiting for a response
// Inputs:     None
// Outputs:    None
// Not used for ASCII mode
void modbus_check_timeout(void)
{

   //modbus_timeout_enabled must be checked before get_ticks()
   //so that if an interrupt happens it cannot be enabled after
   //an old timer value is used in comparison
   if(modbus_timeout_enabled && (get_ticks() > MODBUS_GETDATA_TIMEOUT))
   {
     modbus_timeout_now(); 
   }
}

// Purpose:    Start our timeout timer
// Inputs:     Enable, used to turn timer on/off
// Outputs:    None
// Not used for ASCII mode
void modbus_enable_timeout(int1 enable)
{
   modbus_timeout_enabled = enable;
   set_ticks(0);
}

// Purpose:    Handles a timeout when waiting for a response
// Inputs:     None
// Outputs:    None
// Not used for ASCII mode
void modbus_timeout_now(void)
{
   if((modbus_serial_state == MODBUS_GETDATA) && (modbus_serial_crc.d == 0x0000) && (!modbus_serial_new))
   {
      modbus_rx.len-=2;
      modbus_serial_new=TRUE;
		  //printf("MODBUS DATA:%d %d %d %d %d %d\r\n",modbus_rx.data[0],modbus_rx.data[1],modbus_rx.data[2],modbus_rx.data[3],modbus_rx.data[4],modbus_rx.data[5]);
   } else {
      modbus_serial_new=FALSE;
   }

   modbus_serial_crc.d=0xFFFF;
   modbus_serial_state=MODBUS_GETADDY;
   modbus_enable_timeout(FALSE);
}
// Purpose:    Calculate crc of data and updates global crc
// Inputs:     Character
// Outputs:    None
void modbus_calc_crc(char data)
{
   int8 uIndex ; // will index into CRC lookup table

   uIndex = (modbus_serial_crc.b[1]) ^ data; // calculate the CRC
   modbus_serial_crc.b[1] = (modbus_serial_crc.b[0]) ^ modbus_auchCRCHi[uIndex];
   modbus_serial_crc.b[0] = modbus_auchCRCLo[uIndex];
}

// Purpose:    Puts a character onto the serial line
// Inputs:     Character
// Outputs:    None
void modbus_serial_putc(char c)
{
		// fputc(c, MODBUS_SERIAL);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); // wait for "Transmission Complete" flag cleared
		USART_SendData(USART2,c);
		modbus_calc_crc(c);
		delay_us(1000000/MODBUS_SERIAL_BAUD); //one stop bit.  not exact
}

void incomming_modbus_serial(char c) {

   if (!modbus_serial_new)
   {
      if(modbus_serial_state == MODBUS_GETADDY)
      {
         modbus_serial_crc.d = 0xFFFF;
         modbus_rx.address = c;
         modbus_serial_state++;
         modbus_rx.len = 0;
         modbus_rx.error=0;
      }
      else if(modbus_serial_state == MODBUS_GETFUNC)
      {
         modbus_rx.func = c;
         modbus_serial_state++;
      }
      else if(modbus_serial_state == MODBUS_GETDATA)
      {
         if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE)
       {
         modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE-1;
       }
         modbus_rx.data[modbus_rx.len]=c;
         modbus_rx.len++;
     }
     modbus_enable_timeout(TRUE);
     modbus_calc_crc(c);
   }

   #if (MODBUS_TYPE == MODBUS_TYPE_MASTER)
      modbus_serial_wait=MODBUS_SERIAL_TIMEOUT;
   #endif
}
//------------------------------------------------------------------------------------------------------
void modbus_slave_exe(void)
{
 int8 i,j;
 while(!modbus_kbhit()){};
	//check address against our address, 0 is broadcast
      if((modbus_rx.address == MODBUS_ADDRESS) || modbus_rx.address == 0)
      {
				//printf("FUNC:%d\r\n",modbus_rx.func);
				switch(modbus_rx.func)
         {
					 case FUNC_READ_COILS:    //read coils
					 case FUNC_READ_DISCRETE_INPUT:    //read inputs	 
						 if(modbus_rx.data[0] || modbus_rx.data[2] ||
                  modbus_rx.data[1] >= 8 || modbus_rx.data[3]+modbus_rx.data[1] > 8)
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               else
               {
                  int8 data;

                  if(modbus_rx.func == FUNC_READ_COILS)
                     data = coils>>(modbus_rx.data[1]);      //move to the starting coil
                  else
                     data = inputs>>(modbus_rx.data[1]);      //move to the starting input

                  data = data & (0xFF>>(8-modbus_rx.data[3]));  //0 out values after quantity

                  if(modbus_rx.func == FUNC_READ_COILS)
                     modbus_read_discrete_input_rsp(MODBUS_ADDRESS, 0x01, &data);
                  else
                     modbus_read_discrete_input_rsp(MODBUS_ADDRESS, 0x01, &data);

                  event_count++;
               }
               break;
					case FUNC_READ_HOLDING_REGISTERS: //printf("READ HOLDING REGISTERS\r\n");
          case FUNC_READ_INPUT_REGISTERS:
               if(modbus_rx.data[0] || modbus_rx.data[2] ||
                  modbus_rx.data[1] >= 8 || modbus_rx.data[3]+modbus_rx.data[1] > 8)// user define!!!
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               else
               {
                  if(modbus_rx.func == FUNC_READ_HOLDING_REGISTERS)
									   modbus_read_holding_registers_rsp(MODBUS_ADDRESS,(modbus_rx.data[3]*2),hold_regs+modbus_rx.data[1]);
                  else
										 modbus_read_input_registers_rsp(MODBUS_ADDRESS,(modbus_rx.data[3]*2),input_regs+modbus_rx.data[1]);

                  event_count++;
               }
               break;
					case FUNC_WRITE_SINGLE_COIL:      //write coil
               if(modbus_rx.data[0] || modbus_rx.data[3] || modbus_rx.data[1] > 8)
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               else if(modbus_rx.data[2] != 0xFF && modbus_rx.data[2] != 0x00)
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_VALUE);
               else
               {								 
								 if(modbus_rx.data[2] == 0xFF)
                     bit_set(coils,modbus_rx.data[1]);
                  else
                     bit_clear(coils,modbus_rx.data[1]);

                  modbus_write_single_coil_rsp(MODBUS_ADDRESS,modbus_rx.data[1],((int16)(modbus_rx.data[2]))<<8);

                  event_count++;
               }
							 break;
           case FUNC_WRITE_SINGLE_REGISTER:
               if(modbus_rx.data[0] || modbus_rx.data[1] >= 8)
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               else
               {
                  hold_regs[modbus_rx.data[1]] = make16(modbus_rx.data[2],modbus_rx.data[3]);

                  modbus_write_single_register_rsp(MODBUS_ADDRESS,
                               make16(modbus_rx.data[0],modbus_rx.data[1]),
                               make16(modbus_rx.data[2],modbus_rx.data[3]));
               }
               break;
					 case FUNC_WRITE_MULTIPLE_COILS:
               if(modbus_rx.data[0] || modbus_rx.data[2] ||
                  modbus_rx.data[1] >= 8 || modbus_rx.data[3]+modbus_rx.data[1] > 8)
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               else
               {
                  

                  modbus_rx.data[5] = swap_bits(modbus_rx.data[5]);

                  for(i=modbus_rx.data[1],j=0; i < modbus_rx.data[1]+modbus_rx.data[3]; ++i,++j)
                  {
                     if(bit_test(modbus_rx.data[5],j))
                        bit_set(coils,(7-i));
                     else
                        bit_clear(coils,(7-i));
                  }

                  modbus_write_multiple_coils_rsp(MODBUS_ADDRESS,
                                 make16(modbus_rx.data[0],modbus_rx.data[1]),
                                 make16(modbus_rx.data[2],modbus_rx.data[3]));

                  event_count++;
               }
               break;
            case FUNC_WRITE_MULTIPLE_REGISTERS:
               if(modbus_rx.data[0] || modbus_rx.data[2] ||
                  modbus_rx.data[1] >= 8 || modbus_rx.data[3]+modbus_rx.data[1] > 8)
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               else
               {
                  int i,j;

                  for(i=0,j=5; i < modbus_rx.data[4]/2; ++i,j+=2)
                     hold_regs[i] = make16(modbus_rx.data[j],modbus_rx.data[j+1]);

                  modbus_write_multiple_registers_rsp(MODBUS_ADDRESS,
                                 make16(modbus_rx.data[0],modbus_rx.data[1]),
                                 make16(modbus_rx.data[2],modbus_rx.data[3]));

                  event_count++;
               }
               break;
					default:    //We don't support the function, so return exception
               modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_FUNCTION);
					     printf("We don't support the function, so return exception\r\n");
				 }
			}
}


