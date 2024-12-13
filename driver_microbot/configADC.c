#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "queue.h"

static QueueHandle_t cola_adc_sensor;
uint8_t Fs = 5;  // Sampling frequency in Hz

void ADCTimerInit(void){
    // Calculate the timer period for the desired sampling frequency
    uint32_t ui32Period = SysCtlClockGet() / Fs;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {}
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);

    TimerConfigure(TIMER2_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period - 1);    //Habilitar disparador de ADC por temporizador

    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
    TimerEnable(TIMER2_BASE, TIMER_A);

}

void configADC_IniciaADC(void)
{
    // Configure timer to trigger ADC conversions
    ADCTimerInit();

    // Enable ADC0 and GPIO ports D and E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);
    //HABILITAMOS EL GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    //HABILITAMOS EL GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);

    // Configure GPIO pins for ADC input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //PE0-PE3
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_0);// PD2-PD3

    // Disable sequence 0 during configuration
    ADCSequenceDisable(ADC0_BASE, 0);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 1); //Disparo  (timer trigger)

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH4);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH5);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END); //La ultima muestra provoca la interrupcion

    ADCSequenceEnable(ADC0_BASE, 0); //ACTIVO LA SECUENCIA

    // Enable hardware oversampling
    ADCHardwareOversampleConfigure(ADC0_BASE,64);

    IntPrioritySet(INT_ADC0SS0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    ADCIntRegister(ADC0_BASE, 0, configADC_ISR);
    //Habilitar interrupcion por parte del ADC
    ADCIntEnable(ADC0_BASE, 0);
    IntEnable(INT_ADC0SS0);


    cola_adc_sensor  = xQueueCreate(8, sizeof(MuestrasADCsensor));
    if (cola_adc_sensor  == NULL)
    {
        while (1)
            ;
    }

    IntMasterEnable();
}

void configADC_LeeADC(MuestrasADCsensor *datos)
{
    xQueueReceive(cola_adc_sensor , datos, portMAX_DELAY);
}

void configADC_ISR(void)
{
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    MuestrasLeidasADCsensor leidas;
    MuestrasADCsensor finales;

    // Borra la interrupcion de Timer y del ADC
    ADCIntClear(ADC0_BASE, 0);
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    ADCSequenceDataGet(ADC0_BASE, 0, (uint32_t*) &leidas); //COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, as� que s髄o son significativos los bits del 0 al 11)
    finales.chan1 = leidas.chan1;
    finales.chan2 = leidas.chan2;
    finales.chan3 = leidas.chan3;
    finales.chan4 = leidas.chan4;
    finales.chan5 = leidas.chan5;
    finales.chan6 = leidas.chan6;
    finales.chan7 = leidas.chan7;
    finales.chan8 = leidas.chan8;

    //Guardamos en la cola
    xQueueSendFromISR(cola_adc_sensor , &finales, &higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void CambiarFrecuencia(float freq)
{
//    uint32_t ui32Period = SysCtlClockGet() / Fs;
//    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period - 1); //5 segundo
}
