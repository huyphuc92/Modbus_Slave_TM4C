
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/fpu.h"
#include "utils/ringbuf.h"

#include "modbus_slave.h"


uint16_t test2;
uint16_t test3;
uint16_t test4;
bool bit0, bit1;
tSerial g_tSerialConfig;
tModbusObject g_tStation;

uint16_t ERR;
uint32_t ui32AdcValue;
uint32_t *p1;
uint16_t *p2;
uint16_t *p3;
//*****************************************************************************
//
// System clock
//
//*****************************************************************************
uint32_t g_ui32SysClock;
uint32_t ui32Period;
//-----------------------------------------------------------------------------

void ngat(void)
{

	if(GPIOIntStatus(GPIO_PORTJ_BASE, 1) & GPIO_PIN_0)
	{
		GPIOIntClear(GPIO_PORTJ_BASE,GPIO_PIN_0);
//		WriteCoil(INPUT_COIL, 0, )
	}

	if(GPIOIntStatus(GPIO_PORTJ_BASE, 1) & GPIO_PIN_1)
	{
		GPIOIntClear(GPIO_PORTJ_BASE,GPIO_PIN_1);

	}

}

void ConfigurePort (void)
{
	g_tSerialConfig.ui32PortName = UART0;
	g_tSerialConfig.ui32SysClock = g_ui32SysClock;
	g_tSerialConfig.ui32Baud = 115200;
	g_tSerialConfig.ui8DataBit = 8;
	g_tSerialConfig.ui8StopBit = Stop_1;
	g_tSerialConfig.ui8Parity = NONE;
	g_tSerialConfig.ui16ReponseTime = 0;
	ERR = ConfigSerial(&g_tSerialConfig);
}

void StationConfig(void)
{
	g_tStation.ui16SlaveID = 2;
	g_tStation.ui16NoOf0x = 1000;
	g_tStation.ui16NoOf1x = 1000;
	g_tStation.ui16NoOf3x = 2000;
	g_tStation.ui16NoOf4x = 2000;
	DataFieldRegister((STATUS_COIL | INPUT_COIL | HOLDING_REG | INPUT_REG), &g_tStation);
}

void GPIOConfigure (void)
{

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = GPIO_PIN_7;
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);
}


int
main(void)
{
	FPULazyStackingEnable();
	FPUEnable();
	FPUStackingEnable();
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    g_ui32SysClock = SysCtlClockGet();

    SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_UDMA);
    GPIOConfigure();
    TxRxEnablePin(GPIO_PORTF_BASE, GPIO_PIN_1);
    // Cau hinh cong truyen thong
    ConfigurePort();
// Neu cau hinh that bai thi ko khoi dong modbus
    if(ERR == 0)
    StationConfig();

    while(1)
    {
		BitWriteWord(&test3,4,ReadCoil(STATUS_COIL,0));
		BitWriteWord(&test4,7,ReadCoil(STATUS_COIL,1));

		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, test3);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, test4);


    }
}
