/*
 * dobleBoosterpack.c
 *
 *  Created on: 31 Dec 2022
 *      Author: fraromesc
 */

//LIBRERIAS
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib2.h"
#include "utils/uartstdio.h"
#include "HAL_I2C.h"
#include "sensorlib2.h"

#include "dobleBoosterpack.h"



//___FUNCIONES___
//CONFIGURACION COMPLETA DEL BP y los SENSORES
char BP(uint8_t i)
{
/*  VALOR  RETURN   =>  SENSORES QUE FUNCIONAN
 *          0       =>      Ninguno
 *          1       =>      BMI
 *          2       =>      BME
 *          3       =>      BME & BMI
 *          4       =>      OPT
 *          5       =>      OPT & BMI
 *          6       =>      OPT & BME
 *          7       =>      OPT & BME & BMI
 *          8       =>      TMP
 *          9       =>      TMP & BMI
 *          10      =>      TMP & BME
 *          11      =>      TMP & BME & BMI
 *          12      =>      TMP & OPT
 *          13      =>      TMP & OPT & BMI
 *          14      =>      TMP & OPT & BME
 *          15      =>      TMP & OPT & BME & BMI
 *  Como no tenemos el TMP, el valor deseado debe ser 7.
 */

    int DevID=0;
    uint8_t Sensor_OK=0;
    uint8_t Opt_OK, Tmp_OK, Bme_OK, Bmi_OK;
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

    /*
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
    }*/
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
    return ((Tmp_OK<<3) | (Opt_OK<<2) | (Bme_OK<<1) | Bmi_OK);

}
/*
 * MEDIDAS SENSORES
 */
//bp especifica de que BOOSTERPACK se quiere leer el sensor

//SENSOR DE LUZ
float luz(uint8_t bp)
{
    BP(bp);
    return OPT3001_getLux();
}
int luz_i (uint8_t bp)
{
    BP(bp);
    return (int)round(OPT3001_getLux());
}

//SENSOR BME280 DE PRESION HUMEDAD y TEMPERATURA

// BME280
int returnRslt;
int g_s32ActualTemp   = 0;
unsigned int g_u32ActualPress  = 0;
unsigned int g_u32ActualHumity = 0;

int temp(uint8_t bp)
{
    BP(bp);
    bme280_read_pressure_temperature_humidity(&g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumity);
    return (int)(g_s32ActualTemp/100.0);
}
int pres(uint8_t bp)
{
    BP(bp);
    bme280_read_pressure_temperature_humidity(&g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumity);
    return (int)(g_u32ActualPress/100.0);
}
int hum(uint8_t bp)
{
    BP(bp);
    bme280_read_pressure_temperature_humidity(&g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumity);
    return (int)(g_u32ActualHumity/1000.0);
}
/*      SENSOR BMI160/BMM150
 * No hay funciones para este, pq se actualizan variables globales con define en el header.
 */

//SENSOR DE TEMPERATURA: flotante y entero
void temp007(uint8_t bp, float *Tf_amb, float *Tf_obj)
{
    BP(bp);
    int16_t T_amb, T_obj;
    sensorTmp007Read(&T_amb, &T_obj);
    sensorTmp007Convert(T_amb, T_obj, &Tf_obj, &Tf_amb);
}
void temp007_i(uint8_t bp, int16_t *T_amb, int16_t *T_obj)
{
    BP(bp);
    float Tf_amb, Tf_obj;
    temp007( bp, &Tf_amb, &Tf_obj);
    T_amb = (short)round(Tf_amb);
    T_obj = (short)round(Tf_obj);
}

