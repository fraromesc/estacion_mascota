/*
 * estacionMascota.h
 *
 *  Created on: 31 Dec 2022
 *      Author: fraromesc
 */

#ifndef ESTACIONMASCOTA_H_
#define ESTACIONMASCOTA_H_



void BP(int, int);
#define BP1 BP(1,RELOJ)
#define BP2 BP(2,RELOJ)

float luz(uint8_t);
int luz_i (uint8_t);

void temp007(uint8_t, float *, float *);
void temp007_i(uint8_t, int16_t *, int16_t *);

int temp(uint8_t);
int pres(uint8_t);
int hum(uint8_t);

#define actMag bmi160_bmm150_mag_compensate_xyz(&s_magcompXYZ);
#define actAcel bmi160_read_accel_xyz(&s_accelXYZ);
#define actGiro bmi160_read_gyro_xyz(&s_gyroXYZ);
#endif /* ESTACIONMASCOTA_H_ */
