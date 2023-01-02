/*
 * dobleBoosterpack.h
 *
 *  Created on: 31 Dec 2022
 *      Author: fraromesc
 */
/*^
 * En principio voy a dejar que para cada sensor tenga que especificar que BP
 * se necesita. Preparar define en el main para seleccionar que BP se necesita.
 */

#ifndef DOBLEBOOSTERPACK_H_
#define DOBLEBOOSTERPACK_H_

int RELOJ;                                                          //Variable global para configuración del reloj en el BP


char BP(uint8_t);                                                   //Configuración completa de el Boosterpack
#define BP1 BP(1)                                                   //Abreviatura para BoosterPack 1
#define BP2 BP(2)                                                   //Abreviatura para BoosterPack 2

//SENSORES PARA BOOSTERPACK ACTUAL

float luz(uint8_t);                                                 //Función luz en flotante
int luz_i (uint8_t);                                                //Función lun en entero

//Sensor no habilitado
void temp007(uint8_t, float *, float *);                            //Temperatura en flotante
void temp007_i(uint8_t, int16_t *, int16_t *);                      //Temperatura en entero


int temp(uint8_t);                                                  //Temperatura en entero en ºC
int pres(uint8_t);                                                  //Presion en entero en mBar
int hum(uint8_t);                                                   //Humedad en entero en %


// BMI160/BMM150
int8_t returnValue;
struct bmi160_gyro_t        gyroB1;
struct bmi160_accel_t       accelB1;
struct bmi160_mag_xyz_s32_t magcompB1;

#define actMagB1 bmi160_bmm150_mag_compensate_xyz(&magcompB1)      //Actualiza los datos del compas en variables globales
#define actAcelB1 bmi160_read_accel_xyz(&accelB1)                  //Actializa los datos del acelerómetro en variables globales
#define actGiroB1 bmi160_read_gyro_xyz(&gyroB1)                    //Actualiza los datos del giroscopio en variables globales

struct bmi160_gyro_t        gyroB2;
struct bmi160_accel_t       accelB2;
struct bmi160_mag_xyz_s32_t magcompB2;

#define actMagB2 bmi160_bmm150_mag_compensate_xyz(&magcompB2)      //Actualiza los datos del compas en variables globales
#define actAcelB2 bmi160_read_accel_xyz(&accelB2)                  //Actializa los datos del acelerómetro en variables globales
#define actGiroB2 bmi160_read_gyro_xyz(&gyroB2)                    //Actualiza los datos del giroscopio en variables globales






#endif /* DOBLEBOOSTERPACK_H_ */





