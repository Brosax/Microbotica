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

volatile uint32_t latestADCValue = 0;
volatile uint32_t latestEDGEValue = 0;

//static QueueHandle_t cola_adc_sensor;
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

    // Enable ADC0 and GPIO ports E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

    // Habilitamos el GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);

    // Configure GPIO pins for ADC input (PE1, PE2, PE3)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Disable sequence 1 during configuration
    ADCSequenceDisable(ADC0_BASE, 1);

    // Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    // Configure ADC sequence 1 for timer trigger
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); // Priority 0

    // Configuramos los pasos para PE3 (AIN0), PE2 (AIN1), PE1 (AIN2)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);                      // Step 0 → PE3 (AIN0)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);                      // Step 1 → PE2 (AIN1)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);                      // Step 2 → PE1 (AIN2)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END); // Step 3 → PE1 (AIN2) + End + Interrupt Enable

    // Enable sequence 1
    ADCSequenceEnable(ADC0_BASE, 1);

    // Enable hardware oversampling for better precision
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    // Configura la prioridad e interrupciones
    IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    ADCIntRegister(ADC0_BASE, 1, configADC_ISR); // Registrar ISR para sequencer 1
    ADCIntEnable(ADC0_BASE, 1);                  // Habilitar interrupciones
    IntEnable(INT_ADC0SS1);


//    cola_adc_sensor  = xQueueCreate(8, sizeof(uint32_t));
//    if (cola_adc_sensor  == NULL)
//    {
//        while (1)
//            ;
//    }

    IntMasterEnable();
}

//void configADC_LeeADC(uint32_t *datos)
//{
//    xQueueReceive(cola_adc_sensor , datos, portMAX_DELAY);
//}

//void configADC_ISR(void)
//{
//    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
//
//    uint32_t leidas;
//    uint32_t finales;
//
//    // Borra la interrupcion de Timer y del ADC
//    ADCIntClear(ADC0_BASE, 3);
//    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
//
//    ADCSequenceDataGet(ADC0_BASE, 3, (uint32_t*) &leidas); //COGEMOS LOS DATOS GUARDADOS
//
//    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, as� que s髄o son significativos los bits del 0 al 11)
//    finales = leidas;
//
//    //Guardamos en la cola
//    xQueueSendFromISR(cola_adc_sensor , &finales, &higherPriorityTaskWoken);
//    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
//}
void configADC_ISR(void)
{

    uint32_t leidas[4];
    // Borra la interrupcion de Timer y del ADC
    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, leidas); //COGEMOS LOS DATOS GUARDADOS
    latestADCValue = leidas[0];
    latestEDGEValue = leidas[1];

}


void CambiarFrecuencia(float freq)
{
//    uint32_t ui32Period = SysCtlClockGet() / Fs;
//    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period - 1); //5 segundo
}
