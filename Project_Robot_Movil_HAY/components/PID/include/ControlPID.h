#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"

#define KP 0.3f
#define KI 0.001f
#define Kv 0.03f
#define offset -5
#define VelMax 195
#define VelMin 140
#define Dutyarranque 220

typedef struct _PIDdata
{

    float _input_prev;

    float setpoint;

    // PID factors
    float kP;
    float kI;
    float kD;

    // PID terms
    float Perr;
    float Ierr;
    float Derr;

    // PID terms limits
    float Perrmin;
    float Perrmax;
    float Ierrmin;
    float Ierrmax;
} PIDdata;
typedef PIDdata *ptrPIDdata;

typedef struct
{
    uint8_t button;
    uint8_t state_button;
} Datos;

typedef struct
{
    bool Sensor1;
    bool Sensor2;
} Data_IN;

// extern PIDdata pid_defaults;
// extern ptrPIDdata p_pid_defaults;

extern PIDdata pid_data;
extern ptrPIDdata p_pid_data;

void PID_Init(ptrPIDdata pPd);
void PID_Coefficients(ptrPIDdata pPd, float setpoint, float kP, float kI, float kD);
void PID_SetLimitsPerr(ptrPIDdata pPd, float Perr_min, float Perr_max);
void PID_SetLimitsIerr(ptrPIDdata pPd, float Ierr_min, float Ierr_max);
void PID_ResetIerr(ptrPIDdata pPd);
float PID_Update(ptrPIDdata pPd, float input);

#endif /* PID_H */
