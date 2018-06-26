/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUSDEF_H
#define __MODBUSDEF_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#define int1   uint8_t
#define int8   uint8_t
#define int16  uint16_t
#define int32  uint32_t

#define bit_set(d,b) (d|=(1<<b)) 
#define bit_clear(d,b) (d&=!(1<<b)) 
#define bit_test(a,b) (a & (1<<b))!=0 
#define make16(msb, lsb)  ((msb << 8) | lsb)
uint8_t swap_bits(uint8_t c);
#define _false 0
#define _true  1
#define FALSE 0
#define TRUE  1

#define USE_WITH_PC 1
#define MODBUS_TYPE MODBUS_TYPE_SLAVE
#define MODBUS_SERIAL_TYPE MODBUS_RTU     //use MODBUS_ASCII for ASCII mode
#define MODBUS_SERIAL_RX_BUFFER_SIZE 64
#define MODBUS_SERIAL_BAUD 115200


#define MODBUS_GETDATA_TIMEOUT 40
#define MODBUS_ADDRESS 0x11
//#include "modbus.h"
 #ifndef MODBUS_SERIAL_TIMEOUT
  #if (MODBUS_SERIAL_TYPE == MODBUS_ASCII)
   #define MODBUS_SERIAL_TIMEOUT    1000000
  #else
   #define MODBUS_SERIAL_TIMEOUT      10000     //in us
  #endif
 #endif
//phy layer rtu
void modbus_check_timeout(void);
uint32_t get_ticks(void);
void modbus_enable_timeout(int1 enable);
void modbus_timeout_now(void);

#endif /* __MODBUSDEF_H */

