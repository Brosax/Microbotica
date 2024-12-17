#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "utils/uartstdio.h"
// Define constants for motor identifiers
#define MOTOR_A 0
#define MOTOR_B 1

// Define limits for PWM output
#define MAX_PWM 100
#define MIN_PWM -100


#define RADIO 3.0f  // Wheel radius in cm
#define L 10  //distancia entre las ruedas
#define Convertir_Angulo 57.29f //360.0f / (2.0f * M_PI)
#define Resolucion 20 //en grado
//#define Resolucion 26 //en grado

// PID Controller structure definition
typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float prevError;    // Previous error value
    float integral;     // Integral accumulation
} PIDController;

// Function prototypes for PID control
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd);
float PID_Update(PIDController *pid, float target, float current, float deltaTime);
void Robot_Move_PID(PIDController *pidA, PIDController *pidB, int32_t distance, float deltaTime);
int Rotate_Robot_PID(PIDController *pidA, PIDController *pidB, int32_t angle, float deltaTime);
void SetMotorSpeed(uint8_t motor, int pwmValue);
uint32_t GetEncoderTicks(uint8_t motor);
void SetPID(float kp);
#endif // PID_H
