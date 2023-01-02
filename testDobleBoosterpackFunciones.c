/*
 * Test para ver si funcionan ambos boosterpack
 * author: @fraromesc
 */



#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "driverlib2.h"
#include "utils/uartstdio.h"

#include "HAL_I2C.h"
#include "sensorlib2.h"

#include "dobleBoosterpack.h"


// =======================================================================
// Function Declarations
// =======================================================================






void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Borra flag
    SysCtlDelay(100);
}


int main(void) {


    RELOJ=SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);



    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, RELOJ/2 -1);
    TimerIntRegister(TIMER0_BASE, TIMER_A,Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, RELOJ);
    int l1,l2,t1,t2,p1,p2,h1,h2;
    while(1)
    {
        l1=luz_i(1);
        l2=luz_i(2);
        t1=temp(1);
        t2=temp(2);
        p1=pres(1);
        p2=pres(2);
        h1=hum(1);
        h2=hum(2);

        UARTprintf("\033[10;1H \n      B1    B2");
        UARTprintf("\n Luz   %d    %d",l1, l2);
        UARTprintf("\n Temp  %d    %d", t1, t2);
        UARTprintf("\n Pres  %d    %d", p1, p2);
        UARTprintf("\n Hum   %d    %d", h1, h2);
        actMagB1;
        actGiroB1;
        actAcelB1;
        actMagB2;
        actGiroB2;
        actAcelB2;
        UARTprintf("\n____________________________________");
        UARTprintf("\nAcel:");
        UARTprintf("\n\tB1:\t%d\t%d\t%d", accelB1.x, accelB1.y, accelB1.z);
        UARTprintf("\n\tB2:\t%d\t%d\t%d", accelB2.x, accelB2.y, accelB2.z);
        UARTprintf("\nGiro:");
        UARTprintf("\n\tB1:\t%d\t%d\t%d", gyroB1.x, gyroB1.y, gyroB1.z);
        UARTprintf("\n\tB2:\t%d\t%d\t%d", gyroB2.x, gyroB2.y, gyroB2.z);
        UARTprintf("\nMagCom:");
        UARTprintf("\n\tB1:\t%d\t%d\t%d", magcompB1.x, magcompB1.y, magcompB1.z);
        UARTprintf("\n\tB2:\t%d\t%d\t%d", magcompB2.x, magcompB2.y, magcompB2.z);


    }

    return 0;
}



