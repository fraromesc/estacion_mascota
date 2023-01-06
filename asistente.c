/*
 * asistente.c
 *
 *  Created on: 5 Jan 2023
 *      Author: fraromesc
 */
 
 /*
        TODO
        ->ponerbien MARGENES
        -> Implementar elementos de la placa (Botones, leds, ...)
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
#include "doobleBoosterpack.h"

//ESTADOS
#define reposo                  0
#define mascotaEnPlataforma     1
#define orina                   2
#define limpiarOrina            3
#define excremento              4
#define limpiarExcremento       5
#define comprobacionGominola    6
#define darGominola             7
#define depositoComidaVacio     8
#define depositoComidaRellenado 9
#define depositoAguaVacio       10
#define depositoAguaRellenado   11
#define comederoVacio           12
#define comederoRellenado       13
#define bebederoVacio           14
#define bebederoRellenado       15
//MARGENES
#define maxRellenoDepoAgua 10
#define 
#define  
#define 
#define 
#define 
#define 
#define 
#define 

uint8_t estado = reposo;                    //estado de la FMS
uint32_t presBase = 0; 
uint32_t presBase_ant = 0; 
uint32_t humBase    = 0; 
uint32_t humBase_ant = 0; 
int32_t tempBase = 0; 
int32_t  tempBase_ant = 0; 

uint32_t presComedero = 0; 
uint32_t presComedero_ant = 0; 
uint32_t humBebedero = 0; 
uint32_t humBebedro_ant = 0;
uint32_t contRellenoDepoComida = 0; 
uint32_t contRellenoDepoAgua = 0; 
bool depositoComidaVacio = 0; 
bool depositoAguaVacio = 0;
bool comederoVacio = 0; 
bool bebederoVacio = 0;

 
 


int main ()
{


    while(1)
    {
        switch (estado)
        {
        case reposo:
            if ((presBase_ant-presBase) > margenBaseMascota)
                    estado = mascotaEnPlataforma;
            else if (!depositoAguaVacio && (contRellenoDepoAgua >= maxRellenoDepoAgua))
                    estado = depositoAguaVacio;
            else if (!depositoComidaVacio && (contRellenoDepoComida >= maxRellenoDepoComida))
                    estado = depositoComidaVacion;
            else if ((presComedero < margenPresComedero) && !comderoVacio)
                    estado = comederoVacio;
            else if ((humBebedero < margenHumBebeedro) && !bebederoVacio)
                    estado = bebederoVacio< 
            else if (BOT1)
                    estado = depositoAguaRellenado;
            else if (BOT2)
                    estado = depositoComidaRellenado;
                    
        break; 
        case mascotaEnPlataforma:
            if (((tempBase_ant-tempBase) > margenTempOrina) && (presBase_ant-presBase) > margenPresOrina) && (humBase_ant-humBase) > margenHumOrina))
                estado = orina;
            else if (((tempBase_ant-tempBase) > margenTempExc) && (presBase_ant-presBase) > margenPresExc) && (humBase_ant-humBase) > margenHumExc))
                estado = excremento;
        break; 
        case orina:

        break; 
        case limpiarOrina:

        break; 
        case excremento:

        break; 
        case limpiarExcremento:

        break; 
        case comprobacionGominola:

        break; 
        case darGominola:

        break; 
        case depositoComidaVacio:

        break; 
        case depositoComidaRellenado:

        break; 
        case depositoAguaVacio:

        break; 
        case depositoAguaRellenado:

        break; 
        case comederoVacio:

        break; 
        case comederoRellenado:

        break; 
        case bebederoVacio:

        break; 
        case bebederoRellenado:

        break; 

        default:
        estado = reposo; 
            break;
        }
    }

    return 0; 
}





