/*
 * estacionMascota.h
 *
 *  Created on: 31 Dec 2022
 *      Author: fraromesc
 */
/*^
 * En principio voy a dejar que para cada sensor tenga que especificar que BP
 * se necesita. Preparar define en el main para seleccionar que BP se necesita.
 */

#ifndef ESTACIONMASCOTA_H_
#define ESTACIONMASCOTA_H_

int RELOJ;                                                          //Variable global para configuraci�n del reloj en el BP


char BP(uint8_t);                                                   //Configuraci�n completa de el Boosterpack
#define BP1 BP(1)                                                   //Abreviatura para BoosterPack 1
#define BP2 BP(2)                                                   //Abreviatura para BoosterPack 2

//SENSORES PARA BOOSTERPACK ACTUAL

float luz(uint8_t);                                                 //Funci�n luz en flotante
int luz_i (uint8_t);                                                //Funci�n lun en entero

//Sensor no habilitado
void temp007(uint8_t, float *, float *);                            //Temperatura en flotante
void temp007_i(uint8_t, int16_t *, int16_t *);                      //Temperatura en entero


int temp(uint8_t);                                                  //Temperatura en entero en �C
int pres(uint8_t);                                                  //Presion en entero en mBar
int hum(uint8_t);                                                   //Humedad en entero en %


// BMI160/BMM150
int8_t returnValue;
struct bmi160_gyro_t        gyro;
struct bmi160_accel_t       accel;
struct bmi160_mag_xyz_s32_t magcomp;

#define actMag bmi160_bmm150_mag_compensate_xyz(&magcomp);     //Actualiza los datos del compas en variables globales
#define actAcel bmi160_read_accel_xyz(&accel);                 //Actializa los datos del aceler�metro en variables globales
#define actGiro bmi160_read_gyro_xyz(&gyro);                   //Actualiza los datos del giroscopio en variables globales






#endif /* ESTACIONMASCOTA_H_ */





