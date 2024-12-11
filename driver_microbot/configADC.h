/*
 * configADC.h
 *
 *  Created on: 22/4/2016
 *      Author: jcgar
 */

#ifndef CONFIGADC_H_
#define CONFIGADC_H_

#include<stdint.h>

typedef struct
{
    uint32_t chan1;
    uint32_t chan2;
    uint32_t chan3;
    uint32_t chan4;
    uint32_t chan5;
    uint32_t chan6;
    uint32_t chan7;
    uint32_t chan8;
} MuestrasADCsensor;

typedef struct
{
    uint32_t chan1;
    uint32_t chan2;
    uint32_t chan3;
    uint32_t chan4;
    uint32_t chan5;
    uint32_t chan6;
    uint32_t chan7;
    uint32_t chan8;

} MuestrasLeidasADCsensor;

void ADCTimerInit(void);
void configADC_IniciaADC(void);
void configADC_LeeADC(MuestrasADCsensor *datos);
void configADC_ISR(void);
void CambiarFrecuencia(float freq);

#endif /* CONFIGADC_H_ */
