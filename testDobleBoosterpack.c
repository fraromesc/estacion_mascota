/*
 * Test para ver si funcionan ambos boosterpack
 * author: @fraromesc
 */

//PROBLEMAS AL LEER LOS STRINGS
/*No se porque, el micro se queda parado cuando tiene la combinación:
            sprintf(string,"  OPT3001: %5.3f Lux   \n",lux);
            UARTprintf(string);
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>


#include "driverlib2.h"
#include "utils/uartstdio.h"

#include "HAL_I2C.h"
#include "sensorlib2.h"




// =======================================================================
// Function Declarations
// =======================================================================


int RELOJ;

float lux;
char string[80];
int DevID=0;

int16_t T_amb, T_obj;

 float Tf_obj, Tf_amb;
 int lux_i, T_amb_i, T_obj_i;

 // BME280
 int returnRslt;
 int g_s32ActualTemp   = 0;
 unsigned int g_u32ActualPress  = 0;
 unsigned int g_u32ActualHumity = 0;
// struct bme280_t bme280;

 // BMI160/BMM150
 int8_t returnValue;
 struct bmi160_gyro_t        s_gyroXYZ;
 struct bmi160_accel_t       s_accelXYZ;
 struct bmi160_mag_xyz_s32_t s_magcompXYZ;


 //Calibration off-sets
 int8_t accel_off_x;
 int8_t accel_off_y;
 int8_t accel_off_z;
 int16_t gyro_off_x;
 int16_t gyro_off_y;
 int16_t gyro_off_z;
 float T_act,P_act,H_act;
 bool BME_on = true;

 int T_uncomp,T_comp;
char mode;
long int inicio, tiempo;

volatile long int ticks=0;
uint8_t Sensor_OK=0;
#define BP 2
uint8_t Opt_OK, Tmp_OK, Bme_OK, Bmi_OK;
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
    while(1)
    {
      /*  if(Detecta_BP(1)) UARTprintf("\nBoosterPack 1");
        else if (Detecta_BP(2)) UARTprintf("\nBoosterPack 2");
        else UARTprintf("\n Cagate lorito");
        }
    *////*

    //___CONFIGURACION SENSORES___
    int i;
    for ( i = 1; i < 3; i++)
    {
        UARTprintf("\n\033[10;1H BOOSTERPACK %d \n ", i);
        Conf_Boosterpack(i, RELOJ);
        //Configuracion OPT
        Sensor_OK=Test_I2C_Dir(OPT3001_SLAVE_ADDRESS);
        if(!Sensor_OK)
        {
            UARTprintf("Error en OPT3001\n");
            Opt_OK=0;
        }
        else
        {
            OPT3001_init();
            DevID=OPT3001_readDeviceId();
            Opt_OK=1;
        }
        //Configuracion TMP
        Sensor_OK=Test_I2C_Dir(TMP007_I2C_ADDRESS);
        if(!Sensor_OK)
        {
            UARTprintf("Error  en TMP007\n");
            Tmp_OK=0;
        }
        else
        {
            sensorTmp007Init();
            DevID=sensorTmp007DevID();
            sensorTmp007Enable(true);
            Tmp_OK=1;
        }
        //Configuracion BME
        Sensor_OK=Test_I2C_Dir(BME280_I2C_ADDRESS2);
        if(!Sensor_OK)
        {
            UARTprintf("Error en BME280\n");
            Bme_OK=0;
        }
        else
        {
            bme280_data_readout_template();
            bme280_set_power_mode(BME280_NORMAL_MODE);
            readI2C(BME280_I2C_ADDRESS2,BME280_CHIP_ID_REG, &DevID, 1);
            Bme_OK=1;
        }
        //Configuracion BMI
        Sensor_OK=Test_I2C_Dir(BMI160_I2C_ADDR2);
        if(!Sensor_OK)
        {
            UARTprintf("Error en BMI160\n");
            Bmi_OK=0;
        }
        else
        {
            bmi160_initialize_sensor();
            bmi160_config_running_mode(APPLICATION_NAVIGATION);
            readI2C(BMI160_I2C_ADDR2,BMI160_USER_CHIP_ID_ADDR, &DevID, 1);
            Bmi_OK=1;
        }
        //___MEDIDA DE SENSORES___
        //Medida de OPT
        if(Opt_OK)
        {
            lux=OPT3001_getLux();
            lux_i=(int)round(lux);
            UARTprintf("\n----------------------------\n");
            UARTprintf("  OPT3001: %d Lux   \n",lux_i);
        }
        //Medida de TMP
        if(Tmp_OK)
        {
            sensorTmp007Read(&T_amb, &T_obj);
            sensorTmp007Convert(T_amb, T_obj, &Tf_obj, &Tf_amb);
            T_amb_i=(short)round(Tf_amb);
            T_obj_i=(short)round(Tf_obj);
            UARTprintf("-------------------------------------------\n");
            UARTprintf("  TMP007:  T_a:%d, T_o:%d \n", T_amb_i, T_obj_i);
        }
        //Medida de BME
        if(Bme_OK)
        {
            returnRslt = bme280_read_pressure_temperature_humidity(&g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumity);
            T_act=(int)(g_s32ActualTemp/100.0);
            P_act=(int)(g_u32ActualPress/100.0);
            H_act=(int)(g_u32ActualHumity/1000.0);
            UARTprintf("-----------------------------------------\n");
            UARTprintf("  BME: T:%d C  P:%dmbar  H:%d  \n",T_act,P_act,H_act);
        }
        //Medida de BMI
        if(Bmi_OK)
        {
            bmi160_bmm150_mag_compensate_xyz(&s_magcompXYZ);
            bmi160_read_accel_xyz(&s_accelXYZ);
            bmi160_read_gyro_xyz(&s_gyroXYZ);    
            UARTprintf("------------------------------------------------\n");
            UARTprintf("  BMM:  X:%6d\033[17;22HY:%6d\033[17;35HZ:%6d  \n",s_magcompXYZ.x,s_magcompXYZ.y,s_magcompXYZ.z);
            UARTprintf("------------------------------------------------\n");
            UARTprintf("  ACCL: X:%6d\tY:%6d\tZ:%6d  \n",s_accelXYZ.x,s_accelXYZ.y,s_accelXYZ.z);
            UARTprintf("------------------------------------------------\n");
            UARTprintf("  GYRO: X:%6d\tY:%6d\tZ:%6d  \n",s_gyroXYZ.x,s_gyroXYZ.y,s_gyroXYZ.z);
        }
            //*/
    }

    }

    return 0;
}



