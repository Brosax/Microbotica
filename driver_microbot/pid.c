#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid.h"
#include "PWMLib.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t encoderSemaphoreA,encoderSemaphoreB;

// PID initialization
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
}

// Actualiazar valor PID
float PID_Update(PIDController *pid, float target, float current, float deltaTime) {
    float error = target - current;             // Calculate error
    pid->integral += error * deltaTime;         // Update integral term
    float derivative = (error - pid->prevError) / deltaTime;  // Calculate derivative term
    pid->prevError = error;                     // Update previous error

    // Calculate PID output
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

// Function to move the robot using two PID controllers (for two wheels)
void Robot_Move_PID(PIDController *pidA, PIDController *pidB, int32_t distance,float deltaTime) {

    int32_t targetTicks = ceil(((distance / RADIO) * Convertir_Angulo) / Resolucion);
    //uint32_t targetTicks = (uint32_t)round(((abs(distance) / RADIO) * Convertir_Angulo) / Resolucion);
    int32_t currentTicksA = 0;
    int32_t currentTicksB = 0;

    while ((targetTicks > 0 && (currentTicksA < targetTicks || currentTicksB < targetTicks)) ||
               (targetTicks < 0 && (currentTicksA > targetTicks || currentTicksB > targetTicks))){

        // Compute control output for both wheels
        float outputA = PID_Update(pidA, targetTicks, currentTicksA, deltaTime);
        float outputB = PID_Update(pidB, targetTicks, currentTicksB, deltaTime);

        // configura rango de salida dentro del maximo y minimo permitida
        int pwmA = (int)fminf(fmaxf(outputA, MIN_PWM), MAX_PWM);
        int pwmB = (int)fminf(fmaxf(outputB, MIN_PWM), MAX_PWM);

        // Set motor speeds accordingly
        SetMotorSpeed(MOTOR_A, pwmA);
        SetMotorSpeed(MOTOR_B, pwmB);


        if (targetTicks > 0) {
            currentTicksA += GetEncoderTicks(MOTOR_A);
            if(currentTicksA >= targetTicks){
                SetMotorSpeed(MOTOR_A, 0);
            }
            currentTicksB += GetEncoderTicks(MOTOR_B);
            if(currentTicksB >= targetTicks){
                SetMotorSpeed(MOTOR_B, 0);
            }
        } else {
            currentTicksA -= GetEncoderTicks(MOTOR_A);
            if(currentTicksA <= targetTicks){
                SetMotorSpeed(MOTOR_A, 0);
            }
            currentTicksB -= GetEncoderTicks(MOTOR_B);
            if(currentTicksB <= targetTicks){
                SetMotorSpeed(MOTOR_B, 0);
            }
        }
        // Print debug information
        UARTprintf(" targetTicks : %d, Ticks A: %d, Ticks B: %d, PWM A: %d, PWM B: %d\n", targetTicks, currentTicksA, currentTicksB, pwmA, pwmB);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Stop the robot after reaching the target
    SetMotorSpeed(MOTOR_A, 0);
    SetMotorSpeed(MOTOR_B, 0);
}

// Function to rotate the robot using two PID controllers
int Rotate_Robot_PID(PIDController *pidA, PIDController *pidB, int32_t angle,float deltaTime) {
    int32_t currentTicksA = 0;
    int32_t currentTicksB = 0;
    float thetaRadians = ((angle * M_PI) / 180.0f)*(L / RADIO);
    float targetTicks = ((thetaRadians/2) * Convertir_Angulo)/Resolucion ;

    if (angle > 0) { // Rotate right

        while (abs(targetTicks) > abs(currentTicksA) && abs(targetTicks) > abs(currentTicksB)) {

            // Compute control output for both wheels
            float outputA = PID_Update(pidA, targetTicks, currentTicksA, deltaTime);
            float outputB = PID_Update(pidB, -targetTicks, currentTicksB, deltaTime);

            // Clamp PWM output to be within acceptable range
            int pwmA = (int)fminf(fmaxf(outputA, MIN_PWM), MAX_PWM);
            int pwmB = (int)fminf(fmaxf(outputB, MIN_PWM), MAX_PWM);



            // Set motors to rotate in opposite directions for turning
            SetMotorSpeed(MOTOR_A, pwmA);
            SetMotorSpeed(MOTOR_B, pwmB);

            // Simulate encoder feedback
            currentTicksA += GetEncoderTicks(MOTOR_A);
            currentTicksB -= GetEncoderTicks(MOTOR_B);

            // Print debug information
            UARTprintf("Rotate Right -targetTicks : %d, Ticks A: %d, Ticks B: %d   PWM A: %d, PWM B: %d \n",targetTicks, currentTicksA, currentTicksB, pwmA, pwmB);

            // Add a small delay to prevent too-fast polling
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else if (angle < 0) { // Rotate left
        while (abs(targetTicks) > abs(currentTicksA) && abs(targetTicks) > abs(currentTicksB)) {

            // calculo de salida de pwm
            float outputA = PID_Update(pidA, targetTicks, currentTicksA, deltaTime);
            float outputB = PID_Update(pidB, -targetTicks, currentTicksB, deltaTime);

            //set rango de pwm
            int pwmA = (int)fminf(fmaxf(outputA, MIN_PWM), MAX_PWM);
            int pwmB = (int)fminf(fmaxf(outputB, MIN_PWM), MAX_PWM);

            SetMotorSpeed(MOTOR_A, pwmA);
            SetMotorSpeed(MOTOR_B, pwmB);

            currentTicksA -= GetEncoderTicks(MOTOR_A);
            currentTicksB += GetEncoderTicks(MOTOR_B);

            // Print debug information
            UARTprintf("Rotate Left - Ticks A: %d, Ticks B: %d PWM A: %d, PWM B: %d \n", currentTicksA, currentTicksB, pwmA, pwmB);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Stop the robot after reaching the target
    SetMotorSpeed(MOTOR_A, 0);
    SetMotorSpeed(MOTOR_B, 0);

    return 0;
}




// Function to set motor speed using PWM
void SetMotorSpeed(uint8_t motor, int pwmValue) {
    if(motor==MOTOR_A){
        PWM1Set(pwmValue);
    }else{
        PWM2Set(pwmValue);
    }
    //printf("Motor %d set to PWM %d\n", motor, pwmValue);
}

// Function to get encoder ticks
uint32_t GetEncoderTicks(uint8_t motor) {

    if(motor==MOTOR_A){
        xSemaphoreTake(encoderSemaphoreA, portMAX_DELAY);
    }else{
        xSemaphoreTake(encoderSemaphoreB, portMAX_DELAY);
    }

    return 1;
}
