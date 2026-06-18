/*
 * PID.c
 *
 *  Created on: Nov 12, 2023
 *      Author: eduar
 */
#include "PIDpwm.h"

//definicion de funciones
void PIDpwm_Init(PIDpwm_Controller *pid, float Kp, float Ti, float Td, float min_output, float max_output){
	pid->Kp = Kp;
	pid->Ki = Kp*Ti;
	pid->Kd = Kp*Td;

	//inicializar variables tipo float
	pid->Sp = 0.0f;
    pid->integral = 0.0f;
	pid->errorAnterior = 0.0f;
    pid->medicionSensor_Anterior = 0.0f;
    //limites del PID
    pid->min_output = min_output;
    pid->max_output = max_output;
}

float PIDpwm_Compute(PIDpwm_Controller *pid, float medicionSensor) {
    float error = pid->Sp - medicionSensor;
    pid->integral += error;
    float derivativo = error - pid->errorAnterior;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivativo;

    //saturación de la salida
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;

    //actualizar para la próxima iteración
    pid->errorAnterior = error;
    pid->medicionSensor_Anterior = medicionSensor;

    return output * 255/pid->max_output;
}
