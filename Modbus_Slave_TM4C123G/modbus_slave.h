/*
 * Modbus.h
 *
 *  Created on	: August 6, 2016
 *	Author		: HUYPHUC92
 *	V1.0
 */
#include <stdint.h>
#include <stdbool.h>

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
// UART name
#define UART0              			0x4000C000  // UART0
#define UART1              			0x4000D000  // UART1
#define UART2              			0x4000E000  // UART2
#define UART3              			0x4000F000  // UART3
#define UART4              			0x40010000  // UART4
#define UART5              			0x40011000  // UART5
#define UART6              			0x40012000  // UART6
#define UART7              			0x40013000  // UART7
// Config
#define Stop_1    					0x00000000  // One stop bit
#define Stop_2    					0x00000008  // Two stop bits
//
#define NONE    					0x00000000  // No parity
#define EVEN    					0x00000006  // Even parity
#define ODD    						0x00000002  // Odd parity
#define ONE    						0x00000082  // Parity bit is one
#define ZERO    					0x00000086  // Parity bit is zero
//
#define STATUS_COIL 				0x01
#define INPUT_COIL 					0x02
#define INPUT_REG 					0x08
#define HOLDING_REG 				0x10

#define COILSTATUS_LENGTH 			125			// 250 byte = 2000 coil
#define INPUTSTATUS_LENGTH 			125
#define HOLDINGREGISTER_LENGTH 		200
#define INPUTREGISTER_LENGTH 		200
//*****************************************************************************

typedef struct
{
	//
	// Slave ID
	//
	uint16_t ui16SlaveID;

	//
	// Funcition 0x: Coil status, can be read/write
	//
	uint16_t ui16NoOf0x;

	//
	// Funcition 1x: Input Status, read only
	//
	uint16_t ui16NoOf1x;

	//
	// Funcition 4x: Holding register, can be read/write
	//
	uint16_t ui16NoOf4x;
	//
	// Funcition 3x: Input register, read only
	//
	uint16_t ui16NoOf3x;

}tModbusObject;

typedef struct
{
	//
	// Coil status
	//
	uint16_t pui16CoilStatus[COILSTATUS_LENGTH];

	//
	// Input coil
	//
	uint16_t pui16InputCoil[INPUTSTATUS_LENGTH];

	//
	// Holding register
	//
	uint16_t pui16HoldingRegister[HOLDINGREGISTER_LENGTH];

	//
	// Input Register
	//
	uint16_t pui16InputRegister[INPUTREGISTER_LENGTH];

}tDataField;

typedef struct
{
	//
	// Port name
	//
	uint32_t ui32PortName;

	//
	// System clock
	//
	uint32_t ui32SysClock;

	//
	// Baud rate
	//
	uint32_t ui32Baud;

	//
	// Data bit
	//
	uint8_t ui8DataBit;

	//
	// Parity
	//
	uint8_t ui8Parity;

	//
	// Stop bit
	//
	uint8_t ui8StopBit;

	//
	// Reponse time
	//
	uint16_t ui16ReponseTime;
}tSerial;

//*****************************************************************************
//
// API Funcition prototypes
//
//*****************************************************************************
extern unsigned int ConfigSerial(tSerial* SerialConfig);
extern void DataFieldRegister(uint16_t ui16DataField, tModbusObject* MobbusObject);
extern void DataFieldUnRegister(uint16_t ui16DataField);
extern void TxRxEnablePin(uint32_t ui32Port, uint8_t ui8Pins);
extern bool ReadCoil(uint16_t ui16Fcode, uint16_t ui16Position);
extern void WriteCoil(uint16_t ui16Fcode, uint16_t ui16Position, bool bBitValue);
extern uint16_t ReadWord(uint16_t ui16Fcode, uint16_t ui16Position);
extern uint32_t ReadDWord(uint16_t ui16Fcode, uint16_t ui16Position);
extern void WriteWord(uint16_t ui16Fcode, uint16_t ui16Value, uint16_t ui16Position);
extern void WriteDWord(uint16_t ui16Fcode, uint32_t ui32Value, uint16_t ui16Position);
extern void BitWriteWord(uint16_t * pui8DataDestination, uint16_t ui16Position, bool bBitValue);
//*****************************************************************************
//
// Make the end of the C building for C++ compilers
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
}
#endif
#endif /* MODBUS_SLAVE_H_ */
