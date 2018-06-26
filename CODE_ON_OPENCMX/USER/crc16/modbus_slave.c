//http://www.simplymodbus.ca/FC03.htm
#include "crc16.h"
#include "uart.h"
#include "modbus_define.h"
#include "delay.h"
#include "modbus_phy_layer.h"
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
   int16 hold_regs[] = {0x8822,0x7700,0x6600,0x5500,0x4400,0x3300,0x2200,0x1100};
   int16 input_regs[] = {0x1100,0x2200,0x3300,0x4400,0x5500,0x6600,0x7700,0x8800};
   int16 event_count = 0;
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
   //printf("modbus_exception_rsp\r\n");
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
					     //printf("We don't support the function, so return exception\r\n");
				 }
			}
}


