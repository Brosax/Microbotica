/*
 * PWMLib.h
 *
 *  Created on: 23 oct. 2024
 *      Author: hamed
 */

#ifndef PWMLIB_H_
#define PWMLIB_H_
//#include <stdvoid.h>
#include <stdint.h>

extern void PWMInit();

extern void PWM1Set(int32_t Velocidad);
extern void PWM2Set(int32_t Velocidad);
extern void PWMSetBoth(volatile uint32_t Velocidad1,volatile uint32_t Velocidad2);
extern void Forward();
extern void Back();
extern void right();
extern void left();
extern void stop();

extern void mover_robot(int32_t c);
extern void girar_robot(int32_t g);

#endif /* PWMLIB_H_ */
