/*
 * modbus.c
 *
 *  Created on: August 6, 2016
 *      Author: HUYPHUC92
 *      V1.0
 */
#include "modbus_slave.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/udma.h"
#include "driverlib/fpu.h"
#include "utils/ringbuf.h"
#include "driverlib/debug.h"

//===============================================================================
//*****************************************************************************
//
// Tx Rx enable pin
//
//*****************************************************************************
uint32_t g_ui32TxRxEnablePort;
uint8_t g_ui8TxRxEnablePin;
bool g_bTxRxEnable;
//*****************************************************************************
//
// Transmit busy
//
//*****************************************************************************
bool g_bTransmitBusy;
//*****************************************************************************
//
// Funcition code support
//
//*****************************************************************************
uint16_t g_ui16DataFieldEnable;
uint16_t g_ui16InputCoil;
uint16_t g_ui16StatusCoil;
uint16_t g_ui16InputReg;
uint16_t g_ui16HoldingReg;
//*****************************************************************************
//
// How many byte going to send without CRC checksum byte
//
//*****************************************************************************
uint16_t g_ui16Datalength;
//*****************************************************************************
//
// UART port name
//
//*****************************************************************************
uint32_t g_ui32UARTPort;
//*****************************************************************************
//
// Data mapping
//
//*****************************************************************************
tModbusObject g_tModbus;
tDataField g_tDataField;
//*****************************************************************************
//
// Ringbuffer objects for transmit and receive
//
//*****************************************************************************
tRingBufObject g_rbRemoTIRxRingBuf;
tRingBufObject g_rbRemoTITxRingBuf;
//*****************************************************************************
//
// Ringbuffer size
//
//*****************************************************************************
#define REMOTI_UART_RX_BUF_SIZE 255
#define REMOTI_UART_TX_BUF_SIZE 255
//*****************************************************************************
//
// Ringbuffer size
//
//*****************************************************************************
uint8_t g_pui8RxBuf[REMOTI_UART_RX_BUF_SIZE];
uint8_t g_pui8TxBuf[REMOTI_UART_TX_BUF_SIZE];
//*****************************************************************************
//
// Modbus Reveice buffer
//
//*****************************************************************************
uint8_t g_pui8MbReqPDU[255];
uint8_t g_pui8MBRsp[255];
//===========================================================================================================

//*****************************************************************************
//
// BitwriteByte
// This funcition use to turn on or off a bit in byte
// \Param pui8DataDestination is the byte you want to modify
// \Param ui16Position is the position of bit in byte
// \Param bBitValue is the value of bit will be write
//*****************************************************************************

void BitWriteByte(uint8_t * pui8DataDestination, uint16_t ui16Position, bool bBitValue)
{

	uint8_t Mask1 = 0xFF;
	uint8_t Mask2 = 0x01;
	uint8_t Mask3 = 0x00;
	uint8_t Mask4 = 0xFF;
	Mask3 = (Mask2 << ui16Position) & Mask1;
	Mask4 = (Mask2 << ui16Position) ^ Mask1;
	if(bBitValue)
		*pui8DataDestination = *pui8DataDestination | Mask3 ;
	else
		*pui8DataDestination = *pui8DataDestination & Mask4;

	//
	// End of funcition
	//
}
//*****************************************************************************
//
// BitwriteWord
// This funcition use to turn on or off a bit in Word
// \Param pui8DataDestination is the word you want to modify
// \Param ui16Position is the position of bit in word
// \Param bBitValue is the value of bit will be write
//*****************************************************************************

void BitWriteWord(uint16_t * pui8DataDestination, uint16_t ui16Position, bool bBitValue)
{

	uint16_t Mask1 = 0xFFFF;
	uint16_t Mask2 = 0x01;
	uint16_t Mask3 = 0x00;
	uint16_t Mask4 = 0xFFFF;
	Mask3 = (Mask2 << ui16Position) & Mask1;
	Mask4 = (Mask2 << ui16Position) ^ Mask1;
	if(bBitValue)
		*pui8DataDestination = *pui8DataDestination | Mask3 ;
	else
		*pui8DataDestination = *pui8DataDestination & Mask4;

	//
	// End of funcition
	//
}
//*****************************************************************************
//
// Calculator CRC code
// \This funcition use to genegration CRC checksum code
// \param pui8CRCBuff pointer to Data will be calculate
// \param ui16Length is number of bytes that are to be calculate
// Return CRC code
//
//*****************************************************************************
uint16_t CRCCalculator(uint8_t * pui8CRCbuff,uint16_t ui16Length)
{
	uint16_t ui16CRC=0xFFFF;

	uint16_t i,j;
	for(i=0;i<ui16Length;i++)
	{
		ui16CRC ^= (uint16_t)pui8CRCbuff[i];
		for(j=8;j != 0;j--)
		{
			if((ui16CRC & 0x0001) !=0)
			{
				ui16CRC >>=1;
				ui16CRC ^= 0xA001;
			}
			else
			ui16CRC >>= 1;
		}
	}
	return ui16CRC;

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Check CRC code
// \param pui8DataBuff pointer to Data will be check
// \param ui16Length is number of bytes that are to be check
// Return true if CRC correctly
// Return false if CRC incorrect
//*****************************************************************************
bool CRCCheck(uint8_t* pui8DataBuff, uint16_t ui16Length)
{
	// CRC of Calculator
	uint16_t ui16CRC1=0;
	// CRC of Frame
	uint16_t ui16CRC2= 0;

	ui16CRC1= CRCCalculator(pui8DataBuff, ui16Length-2);
	ui16CRC2 = ui16CRC2 | (uint16_t)(pui8DataBuff[ui16Length-1]);
	ui16CRC2 = ui16CRC2 << 8;
	ui16CRC2 = ui16CRC2 | (uint16_t)(pui8DataBuff[ui16Length-2]);
	return((ui16CRC1 == ui16CRC2) ? true : false);

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Put message to line
// \param pui8MbRsp pointer to Data will be send
// \param ui16Length is number of bytes that are to be send
// Return None
//
//*****************************************************************************
void PutMsg(uint8_t* pui8MbRsp, uint16_t ui16Length)
{

	// CRC code will be adding to frame
	uint16_t ui16CRC;
	// CRC byte in frame
	uint8_t ui8CRCLow, ui8CRCHigh;
	// IntState
	bool bIntState;
	UARTIntDisable(g_ui32UARTPort, UART_INT_RX | UART_INT_RT);
    ui16CRC= CRCCalculator(pui8MbRsp,ui16Length);
	ui8CRCLow = (uint8_t)ui16CRC;
	ui8CRCHigh = (uint8_t)(ui16CRC >> 8);
	pui8MbRsp[ui16Length]=ui8CRCLow;
	pui8MbRsp[ui16Length+1]=ui8CRCHigh;
	//
	// Capture the current state of the master interrupts enable
	//
	bIntState = IntMasterDisable();
	//
	// Store the message in the ringbuffer for transmission
	//
    RingBufWrite(&g_rbRemoTITxRingBuf, pui8MbRsp, (ui16Length + 2));
    //
    // If the UART transmit is idel prime the tranmitter with first byte and enable transmit interrupts
    //
    if(!g_bTransmitBusy)
    {
    	if(g_bTxRxEnable)
    	GPIOPinWrite(g_ui32TxRxEnablePort, g_ui8TxRxEnablePin, g_ui8TxRxEnablePin);
        UARTIntEnable(g_ui32UARTPort, UART_INT_TX);
        UARTCharPutNonBlocking(g_ui32UARTPort, RingBufReadOne(&g_rbRemoTITxRingBuf));

        //
        // Set the transmit busy flag
        //
        g_bTransmitBusy = true;
    }

    //
    // Restore the master interrupts enable to previous state
    //
    if(!bIntState)
    {
    	IntMasterEnable();
    }
    // End of funcition
    //
}

//*****************************************************************************
//
// Process funcition code 01 + 02
// Read multi coil output (Read/Write data field)
// \param pui8MBReadBuff0x is a pointer to coil status memory earea
// \param pui8MBReadBuff1x is a
// \param g_pui16DataCoil this is a buffer content data of system
// Return None
//
//*****************************************************************************
// TODO: Read multi coil
void ReadBit(uint8_t* pui8MBReadBuff, uint16_t* pui16DataBuff0x, uint16_t* pui16DataBuff1x, uint8_t ui8Code)
{
	int i = 0;
	int j = 0;
	int k = 0;

	uint16_t ui16StartAdd;
	uint16_t ui16QuantiData;
	uint16_t ui16StartBit;
	uint16_t ui16StartWord;
	// Byte count
	uint16_t ui16ByteCount = 0;
	// Pointer to send buffer
	uint8_t *p;
	p = g_pui8MBRsp;

	ui16StartAdd = (uint16_t)(pui8MBReadBuff[2]);
	ui16StartAdd = ui16StartAdd << 8;
	ui16StartAdd = ui16StartAdd | ((uint16_t)(pui8MBReadBuff[3]));
	ui16QuantiData = (uint16_t)(pui8MBReadBuff[4]);
	ui16QuantiData = ui16QuantiData << 8;
	ui16QuantiData = ui16QuantiData | ((uint16_t)(pui8MBReadBuff[5]));

	// Check correct start add and quanti data
	if(((ui16StartAdd == 0) || (ui16StartAdd > 0)) && (ui16QuantiData < 2000))
	{
		if(((ui8Code == 1) && ((ui16QuantiData <= g_tModbus.ui16NoOf0x) && (ui16StartAdd <= g_tModbus.ui16NoOf0x) && ((g_tModbus.ui16NoOf0x - ui16StartAdd) >= ui16QuantiData)))
				|| (((ui8Code == 2) && ((ui16QuantiData <= g_tModbus.ui16NoOf1x) && (ui16StartAdd <= g_tModbus.ui16NoOf1x) && ((g_tModbus.ui16NoOf1x - ui16StartAdd) >= ui16QuantiData)))))
		{
			if(ui16StartAdd < 16)
				ui16StartWord = 0;
			else if((ui16StartAdd % 16 == 0) && (ui16StartAdd >16))
				ui16StartWord = (ui16StartAdd / 16) - 1;
			else if((ui16StartAdd % 16 != 0) && (ui16StartAdd >16))
				ui16StartWord = ui16StartAdd / 16;
			if(ui16StartAdd <= 16)
				ui16StartBit = (15 + ui16StartAdd/16) - ui16StartAdd;
			else
				ui16StartBit =16 - (ui16StartAdd - (16*ui16StartWord));

			// Adding Address
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			// Adding Funcition code
			if(ui8Code == 1)
				g_pui8MBRsp[1] = 0x01;
			else
				g_pui8MBRsp[1] = 0x02;
			// Adding byte count
			if(ui16QuantiData % 8 == 0)
				ui16ByteCount = ui16QuantiData / 8;
			else
				ui16ByteCount = (ui16QuantiData / 8) + 1;
			g_pui8MBRsp[2] = (uint8_t)(ui16ByteCount);

			// Adding Data
			p+=3;
			for(i=0;i<ui16QuantiData;i++)
			{
				if(ui8Code == 1)
					BitWriteByte(p,j,(bool)((pui16DataBuff0x[ui16StartWord + k] >> ui16StartBit) & 0x0001));
				else
					BitWriteByte(p,j,(bool)((pui16DataBuff1x[ui16StartWord + k] >> ui16StartBit) & 0x0001));
				j++;
				if(j>7)
				{
					j = 0;
					p++;
				}
				if(ui16StartBit != 0)
					ui16StartBit--;
				else
				{
					k++;
					ui16StartBit = 15;
				}
			}

			g_ui16Datalength = ui16ByteCount + 3;
			//
			// Enable timer wait to putmsg
			//
			IntEnable(INT_TIMER1A);
			TimerEnable(TIMER1_BASE, TIMER_A);

		}
		// Reponse messg
		else
		{
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			if(ui8Code == 1)
				g_pui8MBRsp[1] = 0x81;
			else
				g_pui8MBRsp[2] = 0x02;
			PutMsg(g_pui8MBRsp,3);
		}
	}
	// Reponse messg
	else
	{
		g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
		if(ui8Code == 1)
			g_pui8MBRsp[1] = 0x81;
		else
			g_pui8MBRsp[2] = 0x03;
		PutMsg(g_pui8MBRsp,3);
	}


	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Process funcition code 03 + 04
// Read multi register from data buff (Read/Write data field)
// \param pui8MBReadBuff pointer to buffer content modbus frame read
// \param g_pui16DataBuff this is a buffer content data of system
//	Return None
//
//*****************************************************************************
// TODO: Read multi register
void ReadRegister(uint8_t* pui8MBReadBuff, uint16_t * pui16DataBuff4x, uint16_t * pui16DataBuff3x, uint8_t ui8Code)
{
	int i;
	int j;

	// Start address
	uint16_t ui16StartAdd;

	// No of data to be read
	uint16_t ui16NoAdd;

	ui16StartAdd = (uint16_t)(pui8MBReadBuff[2]);
	ui16StartAdd = ui16StartAdd << 8;
	ui16StartAdd = ui16StartAdd | ((uint16_t)(pui8MBReadBuff[3]));
	ui16NoAdd = (uint16_t)(pui8MBReadBuff[4]);
	ui16NoAdd = ui16NoAdd << 8;
	ui16NoAdd = ui16NoAdd | ((uint16_t)(pui8MBReadBuff[5]));

	// Check correct start add and quanti data
	if(((ui16StartAdd == 0) || (ui16StartAdd > 0)) && (ui16NoAdd <125))
	{
		if(((ui8Code == 3) && ((ui16NoAdd <= g_tModbus.ui16NoOf4x) && (ui16StartAdd <= g_tModbus.ui16NoOf4x) && ((g_tModbus.ui16NoOf4x - ui16StartAdd) >= ui16NoAdd)))
						|| (((ui8Code == 4) && ((ui16NoAdd <= g_tModbus.ui16NoOf3x) && (ui16StartAdd <= g_tModbus.ui16NoOf3x) && ((g_tModbus.ui16NoOf3x - ui16StartAdd) >= ui16NoAdd)))))
		{
			// Adding address
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			if(ui8Code == 3)
			{
				// Adding funcition code
				g_pui8MBRsp[1] = 0x03;
				// Adding byte count
				g_pui8MBRsp[2] = (uint8_t)(ui16NoAdd * 2);
				j = ui16StartAdd;
				for(i=0;i<(ui16NoAdd*2);i+=2)
				{
					g_pui8MBRsp[3+i] = (uint8_t)(pui16DataBuff4x[j] >> 8);
					g_pui8MBRsp[4+i] = (uint8_t)(pui16DataBuff4x[j] & 0x00FF);
					j++;
				}
			}
			else if(ui8Code == 4)
			{
				// Adding funcition code
				g_pui8MBRsp[1] = 0x04;
				// Adding byte count
				g_pui8MBRsp[2] = (uint8_t)(ui16NoAdd * 2);
				j = ui16StartAdd;
				for(i=0;i<(ui16NoAdd*2);i+=2)
				{
					g_pui8MBRsp[3+i] = (uint8_t)(pui16DataBuff3x[j] >> 8);
					g_pui8MBRsp[4+i] = (uint8_t)(pui16DataBuff3x[j] & 0x00FF);
					j++;
				}
			}
			g_ui16Datalength = (ui16NoAdd * 2) + 3;
			//
			// Enable timer wait to putmsg
			//
			IntEnable(INT_TIMER1A);
			TimerEnable(TIMER1_BASE, TIMER_A);

		}

		// Reponse messg
		else
		{
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			if(ui8Code == 3)
				g_pui8MBRsp[1] = 0x83;
			else if(ui8Code == 4)
				g_pui8MBRsp[1] = 0x84;
			g_pui8MBRsp[2] = 0x02;
			PutMsg(g_pui8MBRsp,3);
		}
	}
	// Reponse messg
	else
	{
		g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
		if(ui8Code == 3)
			g_pui8MBRsp[1] = 0x83;
		else if(ui8Code == 4)
			g_pui8MBRsp[1] = 0x84;
		g_pui8MBRsp[2] = 0x03;
		PutMsg(g_pui8MBRsp,3);
	}

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Process funcition code 05
// Write single coil (Read/Write data field)
// \param pui8MBReadBuff pointer to buffer content modbus frame read
// \param g_pui16DataBuff this is a buffer content data of system
//	Return None
//
//*****************************************************************************
// TODO: Fcode05
void Fcode05(uint8_t* pui8MBReadBuff, uint16_t* pui16DataBuff)
{
	uint16_t ui16StartAdd;
	uint16_t ui16StartBit;
	uint16_t ui16StartWord;
	int i = 0;
	// Pointer to send buffer
	uint16_t *p;
	p = pui16DataBuff;
	if((pui8MBReadBuff[4] == 0x00) || (pui8MBReadBuff[4] == 0xFF))
	{
		ui16StartAdd = (uint16_t)(pui8MBReadBuff[2]);
		ui16StartAdd = ui16StartAdd << 8;
		ui16StartAdd = ui16StartAdd | ((uint16_t)(pui8MBReadBuff[3]));
		if(ui16StartAdd <= g_tModbus.ui16NoOf0x)
		{
			if(ui16StartAdd < 16)
				ui16StartWord = 0;
			else if((ui16StartAdd % 16 == 0) && (ui16StartAdd >16))
				ui16StartWord = (ui16StartAdd / 16) - 1;
			else if((ui16StartAdd % 16 != 0) && (ui16StartAdd >16))
				ui16StartWord = ui16StartAdd / 16;
			if(ui16StartAdd <= 16)
				ui16StartBit = (15 + ui16StartAdd/16) - ui16StartAdd;
			else
				ui16StartBit =16 - (ui16StartAdd - (16*ui16StartWord));
			p+=ui16StartWord;
			if(pui8MBReadBuff[4] == 0x00)
				BitWriteWord(p,ui16StartBit,false);
			else if(pui8MBReadBuff[4] == 0xFF)
				BitWriteWord(p,ui16StartBit,true);
			for(i=0;i<6;i++)
			{
				g_pui8MBRsp[i] = g_pui8MbReqPDU[i];
			}
			g_ui16Datalength = 6;
			//
			// Enable timer wait to putmsg
			//
			//IntEnable(INT_TIMER1A);
			//TimerEnable(TIMER1_BASE, TIMER_A);
			IntEnable(INT_TIMER1A);
			TimerEnable(TIMER1_BASE, TIMER_A);
		}
		// Reponse messg
		else
		{
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			g_pui8MBRsp[1] = 0x85;
			g_pui8MBRsp[2] = 0x02;
			PutMsg(g_pui8MBRsp,3);
		}
	}
	// Reponse messg
	else
	{
		g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
		g_pui8MBRsp[1] = 0x85;
		g_pui8MBRsp[2] = 0x03;
		PutMsg(g_pui8MBRsp,3);
	}
	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Process funcition code 06
// Write single register (Read/Write data field)
// \param pui8MBReadBuff pointer to buffer content modbus frame read
// \param g_pui16DataBuff this is a buffer content data of system
//	Return None
//
//*****************************************************************************
// TODO: Fcode06
void Fcode06(uint8_t* pui8MBReadBuff, uint16_t* pui16DataBuff)
{
	int i = 0;
	// Start address
	uint16_t ui16RegisterAdd;

	// No of data to be read
	uint16_t ui16ValueOfRegister;

	uint16_t* p;
	p = pui16DataBuff;
	ui16ValueOfRegister = (uint16_t)(pui8MBReadBuff[4]);
	ui16ValueOfRegister = ui16ValueOfRegister << 8;
	ui16ValueOfRegister = ui16ValueOfRegister | ((uint16_t)(pui8MBReadBuff[5]));
	if(((0 == ui16ValueOfRegister) || (ui16ValueOfRegister > 0)) && (ui16ValueOfRegister <= 0xFFFF))
	{
		ui16RegisterAdd = (uint16_t)(pui8MBReadBuff[2]);
		ui16RegisterAdd = ui16RegisterAdd << 8;
		ui16RegisterAdd = ui16RegisterAdd | ((uint16_t)(pui8MBReadBuff[3]));
		if(ui16RegisterAdd <= g_tModbus.ui16NoOf4x)
		{
			p+= ui16RegisterAdd;
			*p = ui16ValueOfRegister;
			for(i=0;i<6;i++)
			{
				g_pui8MBRsp[i] = g_pui8MbReqPDU[i];
			}
			g_ui16Datalength = 6;
			//
			// Enable timer wait to putmsg
			//
			//IntEnable(INT_TIMER1A);
			//TimerEnable(TIMER1_BASE, TIMER_A);
			IntEnable(INT_TIMER1A);
			TimerEnable(TIMER1_BASE, TIMER_A);
		}
		// Reponse messg
		else
		{
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			g_pui8MBRsp[1] = 0x86;
			g_pui8MBRsp[2] = 0x02;
			PutMsg(g_pui8MBRsp,3);
		}
	}
	// Reponse messg
	else
	{
		g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
		g_pui8MBRsp[1] = 0x86;
		g_pui8MBRsp[2] = 0x03;
		PutMsg(g_pui8MBRsp,3);
	}

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Process funcition code 15
// Write multi coil (Read/Write data field)
// \param pui8MBReadBuff pointer to buffer content modbus frame read
// \param g_pui16DataBuff this is a buffer content data of system
//	Return None
//
//*****************************************************************************
// TODO: Fcode15
void Fcode15(uint8_t* pui8MBReadBuff, uint16_t* pui16DataBuff)
{
	int i = 0;
	int j = 0;
	int k = 0;

	uint16_t ui16StartAdd;
	uint16_t ui16QuantiData;
	uint16_t ui16StartBit;
	uint16_t ui16StartWord;

	// Pointer to send buffer
	uint16_t *p;
	p = pui16DataBuff;

	ui16StartAdd = (uint16_t)(pui8MBReadBuff[2]);
	ui16StartAdd = ui16StartAdd << 8;
	ui16StartAdd = ui16StartAdd | ((uint16_t)(pui8MBReadBuff[3]));
	ui16QuantiData = (uint16_t)(pui8MBReadBuff[4]);
	ui16QuantiData = ui16QuantiData << 8;
	ui16QuantiData = ui16QuantiData | ((uint16_t)(pui8MBReadBuff[5]));
	// Check correct quanti data and byte count
	if((ui16QuantiData >= 1) && (ui16QuantiData <= 1968))
	{
		if((ui16QuantiData <= g_tModbus.ui16NoOf0x) && (ui16StartAdd <= g_tModbus.ui16NoOf0x) && ((g_tModbus.ui16NoOf0x - ui16StartAdd) >= ui16QuantiData))
		{
			if(ui16StartAdd < 16)
				ui16StartWord = 0;
			else if((ui16StartAdd % 16 == 0) && (ui16StartAdd >16))
				ui16StartWord = (ui16StartAdd / 16) - 1;
			else if((ui16StartAdd % 16 != 0) && (ui16StartAdd >16))
				ui16StartWord = ui16StartAdd / 16;
			if(ui16StartAdd <= 16)
				ui16StartBit = (15 + ui16StartAdd/16) - ui16StartAdd;
			else
				ui16StartBit =16 - (ui16StartAdd - (16*ui16StartWord));
			p+=ui16StartWord;
			for(i=0;i<ui16QuantiData;i++)
			{
				BitWriteWord(p,ui16StartBit,(bool)((pui8MBReadBuff[7+j] >> k) & 0x01));
				if(0 != ui16StartBit)
					ui16StartBit--;
				else
				{
					p++;
					ui16StartBit = 15;
				}
				k++;
				if(k>7)
				{
					j++;
					k = 0;
				}
			}

			for(i=0;i<6;i++)
			{
				g_pui8MBRsp[i] = g_pui8MbReqPDU[i];
			}
			g_ui16Datalength = 6;
			//
			// Enable timer wait to putmsg
			//
			IntEnable(INT_TIMER1A);
			TimerEnable(TIMER1_BASE, TIMER_A);

		}
		// Reponse messg
		else
		{
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			g_pui8MBRsp[1] = 0x8F;
			g_pui8MBRsp[2] = 0x02;
			PutMsg(g_pui8MBRsp,3);
		}
	}
	// Reponse messg
	else
	{
		g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
		g_pui8MBRsp[1] = 0x8F;
		g_pui8MBRsp[2] = 0x03;
		PutMsg(g_pui8MBRsp,3);
	}

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Process funcition code 16
// Write milti register (Read/Write data field)
// \param pui8MBReadBuff pointer to buffer content modbus frame read
// \param g_pui16DataCoil this is a buffer content data of system
// Return None
//
//*****************************************************************************
// TODO: Fcode16
void Fcode16(uint8_t * pui8MBReadBuff, uint16_t * pui16Data)
{
	int i = 0;
	int j = 0;
	int k = 0;

	// Start address
	uint16_t ui16StartAdd;

	// No of data to be read
	uint16_t ui16NoAdd;

	// Data buffer callcutale
	uint16_t pui16DataBuff[123];

	ui16NoAdd = (uint16_t)(pui8MBReadBuff[4]);
	ui16NoAdd = ui16NoAdd << 8;
	ui16NoAdd = ui16NoAdd | ((uint16_t)(pui8MBReadBuff[5]));

	if((1 <= ui16NoAdd) && (ui16NoAdd <= 123) && (ui16NoAdd * 2 == (uint16_t)pui8MBReadBuff[6]))
	{

		ui16StartAdd = (uint16_t)(pui8MBReadBuff[2]);
		ui16StartAdd = ui16StartAdd << 8;
		ui16StartAdd = ui16StartAdd | ((uint16_t)(pui8MBReadBuff[3]));
		if((ui16NoAdd <= g_tModbus.ui16NoOf4x) && (ui16StartAdd <= g_tModbus.ui16NoOf4x) && ((g_tModbus.ui16NoOf4x - ui16StartAdd) >= ui16NoAdd))
		{
			//
			// Write data in frame to temporary buffer data
			//
			for(i=0;i<(ui16NoAdd * 2);i+=2)
			{
				pui16DataBuff[j] = (uint16_t)(pui8MBReadBuff[7+i]);
				pui16DataBuff[j] = pui16DataBuff[j] << 8;
				pui16DataBuff[j] = pui16DataBuff[j] | ((uint16_t)(pui8MBReadBuff[8+i]));
				j++;
			}
			//
			// Write temporary buffer data to system data
			//
			for(k=0;k<ui16NoAdd;k++)
			{
				pui16Data[ui16StartAdd + k] = pui16DataBuff[k];
			}
			for(i=0;i<6;i++)
			{
				g_pui8MBRsp[i] = g_pui8MbReqPDU[i];
			}
			g_ui16Datalength = 6;
			//
			// Enable timer wait to putmsg
			//
			IntEnable(INT_TIMER1A);
			TimerEnable(TIMER1_BASE, TIMER_A);

		}
		// Reponse messg
		else
		{
			g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
			g_pui8MBRsp[1] = 0x90;
			g_pui8MBRsp[2] = 0x02;
			PutMsg(g_pui8MBRsp,3);
		}
	}
	else
	{
		g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
		g_pui8MBRsp[1] = 0x90;
		g_pui8MBRsp[2] = 0x03;
		PutMsg(g_pui8MBRsp,3);
	}
	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Data field register
// This funcition use to enable data field
// Return None.
//
//*****************************************************************************
// TODO: DataFieldRegister
void DataFieldRegister(uint16_t ui16DataField, tModbusObject* MobbusObject)
{
	uint16_t Mask = ui16DataField;

	g_ui16DataFieldEnable = Mask;
	g_tModbus.ui16SlaveID = (*MobbusObject).ui16SlaveID;
	g_tModbus.ui16NoOf0x = (*MobbusObject).ui16NoOf0x;
	g_tModbus.ui16NoOf1x = (*MobbusObject).ui16NoOf1x;
	g_tModbus.ui16NoOf3x = (*MobbusObject).ui16NoOf3x;
	g_tModbus.ui16NoOf4x = (*MobbusObject).ui16NoOf4x;

	g_ui16InputCoil = g_ui16DataFieldEnable & INPUT_COIL;
	g_ui16StatusCoil = g_ui16DataFieldEnable & STATUS_COIL;
	g_ui16InputReg = g_ui16DataFieldEnable & INPUT_REG;
	g_ui16HoldingReg = g_ui16DataFieldEnable & HOLDING_REG;
	//
	// End of funcition
	//
}
//*****************************************************************************
//
// UART interrupt handler.
//
// This is the interrupt handler for the UART interrupts from the UART peripheral.
// Return None.
//
//*****************************************************************************
// TODO: UART0IntHandler

void
UARTIntHandler(void)
{
	// Funcition request
	uint16_t 			ui16Fcode = 0;
	uint16_t 			ui16ByteCount = 0;
	static uint16_t 	ui16ReadPos = 0;
	static uint8_t 		ui8TxByte = 0;

	// Interrupt will call back if frame more than 8 byte, so all variable must be static variable
	// Read possition in frame receive.

	do
	{
		//
		// Check if a receive interrupt is pending
		//
		if(UARTIntStatus(g_ui32UARTPort,1) & UART_INT_RX | UART_INT_RT)
		{

			UARTIntClear(g_ui32UARTPort, UART_INT_RX | UART_INT_RT);
			while(UARTCharsAvail(g_ui32UARTPort))
			{
				RingBufWriteOne(&g_rbRemoTIRxRingBuf,UARTCharGetNonBlocking(g_ui32UARTPort));
				g_pui8MbReqPDU[ui16ReadPos]=RingBufReadOne(&g_rbRemoTIRxRingBuf);
				ui16ReadPos++;
				if(ui16ReadPos > 254)
					ui16ReadPos = 0;
			}
			//
			// Determine end of frame.
			//
			ui16Fcode = (uint16_t)g_pui8MbReqPDU[1];

			if(((ui16Fcode == 1) && (g_ui16StatusCoil == STATUS_COIL)) || ((ui16Fcode == 2) && (g_ui16InputCoil == INPUT_COIL))
									|| ((ui16Fcode == 3) && (g_ui16HoldingReg == HOLDING_REG)) || ((ui16Fcode == 4) && (g_ui16InputReg == INPUT_REG))
									|| ((ui16Fcode == 5) && (g_ui16StatusCoil == STATUS_COIL)) || ((ui16Fcode == 6) && (g_ui16HoldingReg == HOLDING_REG)))
			{
				if(ui16ReadPos >= 8)
				{
					//
					// Reset read possition
					//
					ui16ReadPos = 0;

					if((CRCCheck(g_pui8MbReqPDU,8)) && (g_tModbus.ui16SlaveID == (uint16_t)g_pui8MbReqPDU[0]))
					{
						switch(ui16Fcode)
						{
							case 1: //Read multi coil status
							{
								ReadBit(g_pui8MbReqPDU,g_tDataField.pui16CoilStatus,g_tDataField.pui16InputCoil,1);
								break;
							}
							case 2: //Read multi coil input
							{
								ReadBit(g_pui8MbReqPDU,g_tDataField.pui16CoilStatus,g_tDataField.pui16InputCoil,2);
								break;
							}
							case 3: //Read multi holding register
							{
								ReadRegister(g_pui8MbReqPDU, g_tDataField.pui16HoldingRegister, g_tDataField.pui16InputRegister, 3);
								break;
							}
							case 4: //Read input register
							{
								ReadRegister(g_pui8MbReqPDU, g_tDataField.pui16HoldingRegister, g_tDataField.pui16InputRegister, 4);
								break;
							}
							case 5: //Write single coil
							{
								Fcode05(g_pui8MbReqPDU,g_tDataField.pui16CoilStatus);
								break;
							}
							case 6: //Write single register
							{
								Fcode06(g_pui8MbReqPDU,g_tDataField.pui16HoldingRegister);
								break;
							}
						}
					}
				}
			}

			else if(((16 == ui16Fcode) && (g_ui16HoldingReg == HOLDING_REG)) || ((15 == ui16Fcode) && (g_ui16StatusCoil == STATUS_COIL)))
			{
				ui16ByteCount = (uint16_t)g_pui8MbReqPDU[6];
				if((ui16ByteCount + 9) == ui16ReadPos)
				{

					//
					// Reset read possition
					//

					if((CRCCheck(g_pui8MbReqPDU,ui16ReadPos)) && (g_tModbus.ui16SlaveID == (uint16_t)g_pui8MbReqPDU[0]))
					{
						ui16ReadPos = 0;

						switch(ui16Fcode)
						{
							case 15: //Write multi coil
							{
								Fcode15(g_pui8MbReqPDU,g_tDataField.pui16CoilStatus);
								break;
							}
							case 16: //Write multi holding register
							{
								Fcode16(g_pui8MbReqPDU,g_tDataField.pui16HoldingRegister);
								break;
							}
						}

					}

				}
			}
			else
			{
				if(ui16ReadPos > 1)
				{
					//
					// Reset read possition
					//
					ui16ReadPos = 0;
					//
					// Put Exception code
					//

					g_pui8MBRsp[0] = (uint8_t)(g_tModbus.ui16SlaveID);
					g_pui8MBRsp[1] = (uint8_t)(128 + ui16Fcode);
					g_pui8MBRsp[2] = 0x01;
					PutMsg(g_pui8MBRsp,3);
				}
			}

		}

		//
		// Check if a transmit interrupt is pending?
		//
		if(UARTIntStatus(g_ui32UARTPort,1) & UART_INT_TX)
		{
			if(RingBufUsed(&g_rbRemoTITxRingBuf))
			{
				//
			    // We still have more stuff to transfer so read the next byte
			    // from the buffer and load it into the UART.  Finally clear
			    // the pending interrupt status.
			    //
			    UARTIntClear(g_ui32UARTPort, UART_INT_TX);
			    ui8TxByte = RingBufReadOne(&g_rbRemoTITxRingBuf);
			    UARTCharPutNonBlocking(g_ui32UARTPort, ui8TxByte);
			}
			else
			{
				//
			    // Transmission is complete and the internal buffer is empty.
				// Therefore, disable TX interrupts until next transmit is
				// started by the user.
			    //
			    UARTIntClear(g_ui32UARTPort, UART_INT_TX);
			    UARTIntDisable(g_ui32UARTPort, UART_INT_TX);
			    UARTIntEnable(g_ui32UARTPort, UART_INT_RX | UART_INT_RT);
				TimerEnable(TIMER0_BASE, TIMER_A);
				IntEnable(INT_TIMER0A);
			    g_bTransmitBusy = false;
			}
		}
	}
	while(UARTIntStatus(g_ui32UARTPort,1) & (UART_INT_RT | UART_INT_RX | UART_INT_TX));

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Timer0A interrupts
// Use for enable TX/RX pin
//
//*****************************************************************************
void Timer0AIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	if(UARTBusy(g_ui32UARTPort) == false)
	{
		if(g_bTxRxEnable)
		GPIOPinWrite(g_ui32TxRxEnablePort, g_ui8TxRxEnablePin, 0x00);
		IntDisable(INT_TIMER0A);
		TimerDisable(TIMER0_BASE, TIMER_A);
	}

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Timer1A interrupts
// Use for delay reponse time
//
//*****************************************************************************
void Timer1AIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntDisable(INT_TIMER1A);
	TimerDisable(TIMER1_BASE, TIMER_A);
	PutMsg(g_pui8MBRsp,g_ui16Datalength);

	//
	// End of funcition
	//
}
//*****************************************************************************
//
// UART config port
//
// \ Param SerialConfig is a poiter to tSerial structer
// Return None.
//
//*****************************************************************************
// TODO: ConfigSerial
unsigned int ConfigSerial(tSerial* SerialConfig)
{

	uint32_t ui32Config = 0;

//	ASSERT(SerialConfig != NULL);
	if((((*SerialConfig).ui32PortName == UART0) || ((*SerialConfig).ui32PortName == UART1) || ((*SerialConfig).ui32PortName == UART2) || ((*SerialConfig).ui32PortName == UART3)
			|| ((*SerialConfig).ui32PortName == UART4) || ((*SerialConfig).ui32PortName == UART5) || ((*SerialConfig).ui32PortName == UART6) || ((*SerialConfig).ui32PortName == UART7))
			&& ((*SerialConfig).ui32SysClock != 0) && ((*SerialConfig).ui32Baud != 0))
	{
		g_ui32UARTPort = (*SerialConfig).ui32PortName;
		switch ((*SerialConfig).ui32PortName)
		{
	//
	// Config for TM4C1294NCPDT
	//
	#ifdef PART_TM4C1294NCPDT
		case UART0:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
				GPIOPinConfigure(GPIO_PA0_U0RX);
				GPIOPinConfigure(GPIO_PA1_U0TX);
				GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
				break;
			}
		case UART1:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART1);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
				GPIOPinConfigure(GPIO_PB0_U1RX);
				GPIOPinConfigure(GPIO_PB1_U1TX);
				GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
				break;
			}
		case UART2:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART2);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
			    GPIOPinConfigure(GPIO_PA6_U2RX);
			    GPIOPinConfigure(GPIO_PA7_U2TX);
			    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
			    break;
			}
		case UART3:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART3);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
			    GPIOPinConfigure(GPIO_PA4_U3RX);
			    GPIOPinConfigure(GPIO_PA5_U3TX);
			    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
			    break;
			}
		case UART4:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART4);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
			    GPIOPinConfigure(GPIO_PA2_U4RX);
			    GPIOPinConfigure(GPIO_PA3_U4TX);
			    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
			    break;
			}
		case UART5:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART5);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
			    GPIOPinConfigure(GPIO_PC6_U5RX);
			    GPIOPinConfigure(GPIO_PC7_U5TX);
			    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
			    break;
			}
		case UART6:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART6);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOP);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
			    GPIOPinConfigure(GPIO_PP0_U6RX);
			    GPIOPinConfigure(GPIO_PP1_U6TX);
			    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);
			    break;
			}
		case UART7:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART7);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
			    GPIOPinConfigure(GPIO_PC4_U7RX);
			    GPIOPinConfigure(GPIO_PC5_U7TX);
			    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
			    break;
			}
	#endif
	//
	// Config for TM4C123GH6PM
	//
	#ifdef PART_TM4C123GH6PM
		case UART0:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
				GPIOPinConfigure(GPIO_PA0_U0RX);
				GPIOPinConfigure(GPIO_PA1_U0TX);
				GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
				break;
			}
		case UART1:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART1);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
				GPIOPinConfigure(GPIO_PB0_U1RX);
				GPIOPinConfigure(GPIO_PB1_U1TX);
				GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
				break;
			}
		case UART2:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART2);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
				GPIOPinConfigure(GPIO_PD6_U2RX);
				GPIOPinConfigure(GPIO_PD7_U2TX);
				GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
				break;
			}

		case UART3:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART3);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
				GPIOPinConfigure(GPIO_PC6_U3RX);
				GPIOPinConfigure(GPIO_PC7_U3TX);
				GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
				break;
			}
		case UART4:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART4);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
				GPIOPinConfigure(GPIO_PC4_U4RX);
				GPIOPinConfigure(GPIO_PC5_U4TX);
				GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
				break;
			}
		case UART5:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART5);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
				GPIOPinConfigure(GPIO_PE4_U5RX);
				GPIOPinConfigure(GPIO_PE5_U5TX);
				GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
				break;
			}
		case UART6:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART6);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
				GPIOPinConfigure(GPIO_PD4_U6RX);
				GPIOPinConfigure(GPIO_PD5_U6TX);
				GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
				break;
			}
		case UART7:
			{
				SysCtlPeripheralReset(SYSCTL_PERIPH_UART7);
				SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
				GPIOPinConfigure(GPIO_PE0_U7RX);
				GPIOPinConfigure(GPIO_PE1_U7TX);
				GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_0 | GPIO_PIN_1);
				break;
			}
	#endif

		}

		//
		// Config databit
		//
		switch ((*SerialConfig).ui8DataBit)
		{
		case 7:
		{
			ui32Config |= UART_CONFIG_WLEN_7;
			break;
		}
		case 8:
		{
			ui32Config |= UART_CONFIG_WLEN_8;
			break;
		}
		default:
			ui32Config |= UART_CONFIG_WLEN_8;
		}
		//
		// Config parity
		//
		switch((*SerialConfig).ui8Parity)
		{
		case NONE:
		{
			ui32Config |= NONE;
			break;
		}
		case EVEN:
		{
			ui32Config |= EVEN;
			break;
		}
		case ODD:
		{
			ui32Config |= ODD;
			break;
		}
		case ONE:
		{
			ui32Config |= ONE;
			break;
		}
		case ZERO:
		{
			ui32Config |= ZERO;
			break;
		}
		}
		//
		// Config Stopbit
		//
		switch((*SerialConfig).ui8StopBit)
		{
		case Stop_1:
		{
			ui32Config |= Stop_1;
			break;
		}
		case Stop_2:
		{
			ui32Config |= Stop_2;
			break;
		}
		}

		//
		// Config port
		//
		UARTConfigSetExpClk((*SerialConfig).ui32PortName, (*SerialConfig).ui32SysClock, (*SerialConfig).ui32Baud, ui32Config);
	    UARTFIFOLevelSet((*SerialConfig).ui32PortName, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
	    UARTEnable((*SerialConfig).ui32PortName);
	    UARTFIFODisable((*SerialConfig).ui32PortName);
		UARTIntRegister((*SerialConfig).ui32PortName, &UARTIntHandler);
		UARTIntEnable((*SerialConfig).ui32PortName, UART_INT_RX|UART_INT_RT);
	    RingBufInit(&g_rbRemoTIRxRingBuf, g_pui8RxBuf, REMOTI_UART_RX_BUF_SIZE);
	    RingBufInit(&g_rbRemoTITxRingBuf, g_pui8TxBuf, REMOTI_UART_TX_BUF_SIZE);

	    //
	    // Config timer for enable TX/RX pin
	    //
	    SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() / 10000) );
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);

	    //
	    // Config timer for delay reponse time
	    //
		SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
		TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
		TimerLoadSet(TIMER1_BASE, TIMER_A, 50000 * ((*SerialConfig).ui16ReponseTime));
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
		TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1AIntHandler);

		//
		//
		//
		IntMasterEnable();

	}

	//
	// Return err code
	//
	else
		return 1;
	return 0;

    //
    // End of funcition
    //

}


//*****************************************************************************
//
// DataFieldUnRegister
// This funcition use to disable data field
// Return None.
//
//*****************************************************************************
void DataFieldUnRegister(uint16_t ui16DataField)
{
	uint16_t ui16Mask1 = 0x01;
	uint16_t ui16Mask2 = 0xFFFF;
	bool 	bBitValue = false;
	int i;
	for(i=0;i<5;i++)
	{
		bBitValue = (bool)((ui16DataField >> i) & 0x0001);
		ui16Mask1 = ui16Mask1 << i;
		ui16Mask2 = ui16Mask2 & ui16Mask1;
		if(bBitValue)
			g_ui16DataFieldEnable = g_ui16DataFieldEnable ^ ui16Mask2;
		ui16Mask2 = 0xFFFF;
		ui16Mask1 = 0x01;
	}

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// TxRxEnablePin
// This funcition use to read value in word of data field
// \Pram ui16Fcode is a name of data field
// Return data contain in register
//
//*****************************************************************************
void TxRxEnablePin(uint32_t ui32Port, uint8_t ui8Pins)
{
	g_ui32TxRxEnablePort = ui32Port;
	g_ui8TxRxEnablePin = ui8Pins;
	g_bTxRxEnable = true;
}
//*****************************************************************************
//
// ReadWord
// This funcition use to read value in word of data field
// \Pram ui16Fcode is a name of data field
// Return data contain in register
//
//*****************************************************************************
// TODO: Read register
uint16_t ReadWord(uint16_t ui16Fcode, uint16_t ui16Position)
{
	uint16_t ui16DataReturn = 0;

	switch (ui16Fcode)
	{
	case INPUT_COIL:
	{
		ui16DataReturn = g_tDataField.pui16InputCoil[ui16Position];
		break;
	}
	case STATUS_COIL:
	{
		ui16DataReturn = g_tDataField.pui16CoilStatus[ui16Position];
		break;
	}
	case INPUT_REG:
	{
		ui16DataReturn = g_tDataField.pui16InputRegister[ui16Position];
		break;
	}
	case HOLDING_REG:
	{
		ui16DataReturn = g_tDataField.pui16HoldingRegister[ui16Position];
		break;
	}
	}
	return ui16DataReturn;

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// ReadWord
// This funcition use to read value in word of data field
// \Pram ui16Fcode is a name of data field
// Return data contain in register
//
//*****************************************************************************
uint32_t ReadDWord(uint16_t ui16Fcode, uint16_t ui16Position)
{
	uint32_t ui32DataReturn = 0;
	uint32_t ui32Mask = 0;

	switch (ui16Fcode)
		{
		case INPUT_COIL:
		{
			ui32DataReturn = (uint16_t)(g_tDataField.pui16InputCoil[ui16Position]);
			ui32Mask = (uint16_t)(g_tDataField.pui16InputCoil[ui16Position + 1]);
			ui32Mask = ui32Mask << 16;
			ui32DataReturn = ui32DataReturn | ui32Mask;
			break;
		}
		case STATUS_COIL:
		{
			ui32DataReturn = (uint16_t)(g_tDataField.pui16CoilStatus[ui16Position]);
			ui32Mask = (uint16_t)(g_tDataField.pui16CoilStatus[ui16Position + 1]);
			ui32Mask = ui32Mask << 16;
			ui32DataReturn = ui32DataReturn | ui32Mask;
			break;
		}
		case INPUT_REG:
		{
			ui32DataReturn = (uint16_t)(g_tDataField.pui16InputRegister[ui16Position]);
			ui32Mask = (uint16_t)(g_tDataField.pui16InputRegister[ui16Position + 1]);
			ui32Mask = ui32Mask << 16;
			ui32DataReturn = ui32DataReturn | ui32Mask;
			break;
		}
		case HOLDING_REG:
		{
			ui32DataReturn = (uint16_t)(g_tDataField.pui16HoldingRegister[ui16Position]);
			ui32Mask = (uint16_t)(g_tDataField.pui16HoldingRegister[ui16Position + 1]);
			ui32Mask = ui32Mask << 16;
			ui32DataReturn = ui32DataReturn | ui32Mask;
			break;
		}
		}
	return ui32DataReturn;

	//
	// End of funcition
	//
}
//*****************************************************************************
//
// WriteRegister
// This funcition use to write a value to word of data field
// \Param ui16Fcode is a name of data field
// \Param ui16Value is a value write to distination
// \Param ui16Position is a distination of word will be writed
// Return None.
//
//*****************************************************************************
// TODO: WriteRegister
void WriteWord(uint16_t ui16Fcode, uint16_t ui16Value, uint16_t ui16Position)
{
	switch (ui16Fcode)
	{
	case INPUT_COIL:
	{
		g_tDataField.pui16InputCoil[ui16Position] = ui16Value;
		break;
	}
	case STATUS_COIL:
	{
		g_tDataField.pui16CoilStatus[ui16Position] = ui16Value;
		break;
	}
	case INPUT_REG:
	{
		g_tDataField.pui16InputRegister[ui16Position] = ui16Value;
		break;
	}
	case HOLDING_REG:
	{
		g_tDataField.pui16HoldingRegister[ui16Position] = ui16Value;
		break;
	}
	}

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// WriteDWord
// This funcition use to read value in word of data field
// \Pram ui16Fcode is a name of data field
// \Param ui32Value is a value write to distination
// \Param ui16Position is a distination of word in data field
// Return data contain in register
//
//*****************************************************************************
void WriteDWord(uint16_t ui16Fcode, uint32_t ui32Value, uint16_t ui16Position)
{
	uint16_t ui16ValueH = 0;
	uint16_t ui16ValueL = 0;
	//
	ui16ValueH = (uint16_t)(ui32Value & 0xFFFF);
	ui16ValueL = (uint16_t)((ui32Value >> 16) & 0xFFFF);
	switch (ui16Fcode)
	{
	case INPUT_COIL:
	{
		g_tDataField.pui16InputCoil[ui16Position] = ui16ValueH;
		g_tDataField.pui16InputCoil[ui16Position+ 1] = ui16ValueL;
		break;
	}
	case STATUS_COIL:
	{
		g_tDataField.pui16CoilStatus[ui16Position] = ui16ValueH;
		g_tDataField.pui16CoilStatus[ui16Position + 1] = ui16ValueL;
		break;
	}
	case INPUT_REG:
	{
		g_tDataField.pui16InputRegister[ui16Position] = ui16ValueH;
		g_tDataField.pui16InputRegister[ui16Position + 1] = ui16ValueL;
		break;
	}
	case HOLDING_REG:
	{
		g_tDataField.pui16HoldingRegister[ui16Position] = ui16ValueH;
		g_tDataField.pui16HoldingRegister[ui16Position + 1] = ui16ValueL;
		break;
	}
	}

	//
	// End of funcition
	//
}
//*****************************************************************************
//
// Read coil
// This funcition use to read a coil in word of data field
// \Param ui16Fcode is a name of data field
// \Param ui16Position is a position of bit in data field
// Return value of bit
//
//*****************************************************************************
// TODO: Read coil
bool ReadCoil(uint16_t ui16Fcode, uint16_t ui16Position)
{
	bool 		bBitValue = false;
	uint16_t 	ui16StartWord = 0;
	uint16_t 	ui16StartBit = 0;

	if(ui16Position < 16)
		ui16StartWord = 0;
	else if((ui16Position % 16 == 0) && (ui16Position >16))
		ui16StartWord = (ui16Position / 16) - 1;
	else if((ui16Position % 16 != 0) && (ui16Position >16))
		ui16StartWord = ui16Position / 16;
	if(ui16Position <= 16)
		ui16StartBit = (15 + ui16Position/16) - ui16Position;
	else
		ui16StartBit =16 - (ui16Position - (16*ui16StartWord));
	switch (ui16Fcode)
		{
		case INPUT_COIL:
		{
			bBitValue =(bool)((g_tDataField.pui16InputCoil[ui16StartWord] >> ui16StartBit) & 0x0001);
			break;
		}
		case STATUS_COIL:
		{
			bBitValue =(bool)((g_tDataField.pui16CoilStatus[ui16StartWord] >> ui16StartBit) & 0x0001);
			break;
		}
		case INPUT_REG:
		{
			bBitValue =(bool)((g_tDataField.pui16InputRegister[ui16StartWord] >> ui16StartBit) & 0x0001);
			break;
		}
		case HOLDING_REG:
		{
			bBitValue =(bool)((g_tDataField.pui16HoldingRegister[ui16StartWord] >> ui16StartBit) & 0x0001);
			break;
		}
		}

	return bBitValue;

	//
	// End of funcition
	//
}

//*****************************************************************************
//
// Write coil
// This funcition use to turn on or off a bit in word of data field
// \Parma ui16Fcode is a name of data field
// \Param ui16Position is a position of bit in data field
// \Param bBitValue is a value will be write to distination
// Return None.
//
//*****************************************************************************
// TODO: Write coil
void WriteCoil(uint16_t ui16Fcode, uint16_t ui16Position, bool bBitValue)
{
	uint16_t ui16StartWord = 0;
	uint16_t ui16StartBit = 0;
	uint16_t Mask1 = 0xFFFF;
	uint16_t Mask2 = 0x01;
	uint16_t Mask3 = 0x00;
	uint16_t Mask4 = 0xFFFF;

	if(ui16Position < 16)
		ui16StartWord = 0;
	else if((ui16Position % 16 == 0) && (ui16Position >16))
		ui16StartWord = (ui16Position / 16) - 1;
	else if((ui16Position % 16 != 0) && (ui16Position >16))
		ui16StartWord = ui16Position / 16;
	if(ui16Position <= 16)
		ui16StartBit = (15 + ui16Position/16) - ui16Position;
	else
		ui16StartBit =16 - (ui16Position - (16*ui16StartWord));

	Mask3 = (Mask2 << ui16StartBit) & Mask1;
	Mask4 = (Mask2 << ui16StartBit) ^ Mask1;
	switch (ui16Fcode)
		{
		case INPUT_COIL:
		{
			if(bBitValue)
				g_tDataField.pui16InputCoil[ui16StartWord] = g_tDataField.pui16InputCoil[ui16StartWord] | Mask3 ;
			else
				g_tDataField.pui16InputCoil[ui16StartWord] = g_tDataField.pui16InputCoil[ui16StartWord] & Mask4 ;
			break;
		}
		case STATUS_COIL:
		{
			if(bBitValue)
				g_tDataField.pui16CoilStatus[ui16StartWord] = g_tDataField.pui16CoilStatus[ui16StartWord] | Mask3 ;
			else
				g_tDataField.pui16CoilStatus[ui16StartWord] = g_tDataField.pui16CoilStatus[ui16StartWord] & Mask4 ;
			break;
		}
		case INPUT_REG:
		{
			if(bBitValue)
				g_tDataField.pui16InputRegister[ui16StartWord] = g_tDataField.pui16InputRegister[ui16StartWord] | Mask3 ;
			else
				g_tDataField.pui16InputRegister[ui16StartWord] = g_tDataField.pui16InputRegister[ui16StartWord] & Mask4 ;
			break;
		}
		case HOLDING_REG:
		{
			if(bBitValue)
				g_tDataField.pui16HoldingRegister[ui16StartWord] = g_tDataField.pui16HoldingRegister[ui16StartWord] | Mask3 ;
			else
				g_tDataField.pui16HoldingRegister[ui16StartWord] = g_tDataField.pui16HoldingRegister[ui16StartWord] & Mask4 ;
			break;
		}
		}

	//
	// End of funcition
	//
}
