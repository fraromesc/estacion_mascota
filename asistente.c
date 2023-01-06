/*
 * asistente.c
 *
 *  Created on: 5 Jan 2023
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
#define 
#define 
#define  
#define 
#define 
#define 
#define 
#define 
#define 

uint8_t estado = reposo;                    //estado de la FMS
int
int main ()
{


    while(1)
    {
        switch (estado)
        {
        case reposo:

        break; 
        case mascotaEnPlataforma:

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





