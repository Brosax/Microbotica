/*################################################
 * PWMLib.h
 *
 *  Created on: 23 oct. 2024
 *      Author: hamed
 *
#################################################*/

#include<math.h>
#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "pid.h"
//#include "PWMLib.h"


#define RADIO 2.9f  // Wheel radius in cm
#define L 10  //distancia entre las ruedas
#define Convertir_Angulo (360.0f / (2.0f * M_PI))
#define Resolucion 20 //en grado

extern SemaphoreHandle_t encoderSemaphoreA;
extern PIDController pidA, pidB;

void PWMInit()
{

    //Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    //Configure PF1,PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE,  GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc

    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks) 20ms
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 15625 );
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 15625 );

    //Set PWM duty-50% motor en reposo  1.5ms 1192
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,392);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)*0.075);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)*0.075);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE,  PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

}
void PWM3Set(volatile int32_t Active)
{
    if(Active==1){
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,1992);
    }else{
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,392);
    }
}

void PWM1Set(volatile int32_t Velocidad)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,((Velocidad*8)+1192));
}
void PWM2Set(volatile int32_t Velocidad)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,((Velocidad*-8)+1192));
}
void PWMSetBoth(volatile int32_t Velocidad1,volatile int32_t Velocidad2)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,((Velocidad1*4)+1192));
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,((Velocidad2*-4)+1192));
}
void Forward()
{
    PWM1Set(100);
    PWM2Set(100);
}

void Back()
{
    PWM1Set(-100);
    PWM2Set(-100);
}
void right()
{
    PWM1Set(0);
    PWM2Set(50);

}
void left()
{
    PWM1Set(50);
    PWM2Set(0);
}
void stop()
{
    PWM1Set(0);
    PWM2Set(0);
}

int mover_robot(float c) {

    Robot_Move_PID(&pidA, &pidB,c,0);

    return 0;
}

int girar_robot(float g)
{
    Rotate_Robot_PID(&pidA, &pidB,g,0);

    return 0;
}
