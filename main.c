#include<stdbool.h>
#include<stdint.h>
#include<math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "commands.h"

#include <driver_microbot/PWMLib.h>
#include <driver_microbot/pid.h>
#include <driver_microbot/configADC.h>

//parametros de funcionamiento de la tareas
#define COMMAND_TASK_STACK (512)
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define ADC_TASK_STACK (512)
#define ADC_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define SW_TASK_PRIO (tskIDLE_PRIORITY+1)           // Prioridad para la tarea SW1TASK
#define SW_TASK_STACK_SIZE (256)                    // Tamanio de pila para la tarea SW1TASK


#define RADIO 3.0f  // Wheel radius in cm
#define Convertir_Angulo 57.29f //360.0f / (2.0f * M_PI)
#define Resolucion 20 //en grado
#define L 10    //Separaci¨®n entre ruedas
// 表大小
#define TABLE_SIZE (sizeof(lookupTable) / sizeof(LookupTableEntry))
// 定义查找表的结构
typedef struct {
    int32_t distancia;    // 距离
    int32_t adcValue;     // ADC 值
} LookupTableEntry;

// 定义查找表
const LookupTableEntry lookupTable[] = {
    {3, 3600},
    {4, 3400},
    {5, 2857},
    {6, 2199},
    {7, 2000},
    {8, 1750},
    {9, 1550},
    {10, 1449},
    {11, 1300},
    {12, 1200},
    {13, 1100},
    {14, 1023},
    {15, 950},
    {16, 905},
    {18, 806},
    {20, 708},
    {22, 632},
    {24, 584},
    {26, 539},
    {30, 450},
    {35, 337},
    {40, 294},
};






//Globales
volatile uint32_t g_ui32CPUUsage;
volatile uint32_t g_ulSystemClock;
volatile uint32_t g_ulSystemClock2;

PIDController pidA, pidB;


//float x = 0.5;  // Valor X del joystick
//float y = 0.3;  // Valor Y del joystick
//
//int motor1 = 0;
//int motor2 = 0;



SemaphoreHandle_t miSemaforo,miSemaforo2,FrontEdgeSemaphore;
SemaphoreHandle_t encoderSemaphoreA,encoderSemaphoreB,BarraSemaphore;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}

void Setup_Hardware(void){
    //Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 4)
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // Get the system clock speed.
    g_ulSystemClock = SysCtlClockGet();

    //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);
    //Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1
    RGBInit(1);
    MAP_SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
    MAP_SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
    MAP_SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);  //Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Volvemos a configurar los LEDs en modo GPIO POR Defecto
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    //MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    ButtonsInit();
    MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);//para añadir prioridad by HAMED
    MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS|GPIO_PIN_4);
    MAP_IntEnable(INT_GPIOF);

    // Configure Port A for encoder input
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_DIR_MODE_IN);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7);
    MAP_IntPrioritySet(INT_GPIOA,configMAX_SYSCALL_INTERRUPT_PRIORITY);//para añadir prioridad by HAMED

    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_BOTH_EDGES); // Configure interrupt on both rising and falling edges



    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_RISING_EDGE); // Configure interrupt rising edges ,barra frontal.

    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_HIGH_LEVEL);

    //MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_RISING_EDGE); // sensor suelo frontal

    MAP_GPIOIntEnable(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7);
    MAP_IntEnable(INT_GPIOA);


    // 配置定时器
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);


    miSemaforo  = xSemaphoreCreateBinary();
    miSemaforo2 = xSemaphoreCreateBinary();
    FrontEdgeSemaphore = xSemaphoreCreateBinary();
    encoderSemaphoreA = xSemaphoreCreateBinary();
    encoderSemaphoreB = xSemaphoreCreateBinary();
    BarraSemaphore = xSemaphoreCreateBinary();

    if (encoderSemaphoreA == NULL || encoderSemaphoreB == NULL || BarraSemaphore == NULL ||miSemaforo == NULL || miSemaforo2 == NULL || FrontEdgeSemaphore == NULL ) {
       while (1);
    }


}

//int mover_robotM(int32_t distance)
//{
//    float targetTicks = ceil(((distance / RADIO )* Convertir_Angulo)/Resolucion);
//
//    int currentTicksA = 0;
//    int currentTicksB = 0;
//
//    // Decide direction and move
//    if (distance > 0) {
//        Forward();
//    } else if (distance < 0) {
//        Back();
//    }
//
//    while (abs(targetTicks) > currentTicksA && abs(targetTicks) > currentTicksB) {
//
//        xSemaphoreTake(encoderSemaphoreA, portMAX_DELAY);
//        currentTicksA++;
//
//        xSemaphoreTake(encoderSemaphoreB, portMAX_DELAY);
//        currentTicksB++;
//
//        UARTprintf("A = %d ,B = %d \r\n",currentTicksA,currentTicksB);
//       }
//    stop();
//
//    return 0;
//}



int lazocerado()
{
    mover_robot(15);

    girar_robot(-95);

    mover_robot(15);

    girar_robot(-95);

    mover_robot(15);

    girar_robot(-95);

    mover_robot(15);

    girar_robot(-95);

    return 0;
}
//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************


//Funcion de leer sensor de distancia y enciende led
//static portTASK_FUNCTION(ADCTask,pvParameters)
//{
//
//    MuestrasADCsensor muestras;
//  //  MESSAGE_ADC_SAMPLE_PARAMETER parameter;
//    int distancia = 1110 ;
//
//    //
//    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
//    //
//    while(1)
//    {
//        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
//
//
//
//        if( muestras.chan2 < 3420 && muestras.chan2 > 899)
//        {
//            distancia = -(muestras.chan2 - 3614) / 179.08;
//        }
//
//        else if( muestras.chan2 < 900 && muestras.chan2 > 286)
//        {
//            distancia = -(muestras.chan2 - 1222.6) / 24.853;
//        }
//        else
//        {
//            distancia = 111111;
//        }
//
//        UARTprintf("distancia %d RAW %d \r\n",distancia ,muestras.chan2);
//
//
//        if (distancia >= 5 && distancia < 10 )
//        {
//            // cm se enciende el led verde PF3
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00000008);
//        }
//        else if (distancia >= 10 && distancia < 15 )
//        {
//                    // cm se enciende el led rojo PF1
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00000002);
//
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0x00000002);
//
//        }
//        else if (distancia >= 15 && distancia <= 20 )
//        {
//                    //  cm se encienden ambos leds rojo y verde
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 ,0x00000002);
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,0x00000008);
//        }
//        else
//        {
//            //los leds permanecen apagados
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);
//        }
//
//    }
//}

static portTASK_FUNCTION(ADCTask, pvParameters) {
    MuestrasADCsensor muestras;
    int distancia = 111111;

    while (1) {
        configADC_LeeADC(&muestras);  // 获取ADC结果

        // 遍历映射表，找到匹配的范围
        int i = 0;
        for (i = 0; i < TABLE_SIZE; i++)
        {
            if (muestras.chan2 >= lookupTable[i].adcValue) {
                distancia = lookupTable[i].distancia;
                break;
            }
        }

        UARTprintf("distancia %d RAW %d \r\n", distancia, muestras.chan2);

        // 基于距离设置LED状态
        if (distancia >= 5 && distancia < 10) {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00000008);  // 绿色LED
        } else if (distancia >= 10 && distancia < 15) {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00000002);  // 红色LED
        } else if (distancia >= 15 && distancia <= 20) {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00000002);  // 红色LED
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00000008);  // 绿色LED
        } else {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);  // 关闭LED
        }
    }
}




//static void Switch1Task(void *pvParameters)
static portTASK_FUNCTION(Switch1Task,pvParameters)
{
    TickType_t xLastWakeTime ;
    //
    // Loop forever.
    //
    while(1)
    {
        xSemaphoreTake(miSemaforo,portMAX_DELAY);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        mover_robot(15);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        girar_robot(-90);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        mover_robot(15);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        girar_robot(-90);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        mover_robot(15);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        girar_robot(-90);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        mover_robot(15);
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        girar_robot(-90);




    }
}

static portTASK_FUNCTION(Switch2Task,pvParameters)
{
    //
    // Loop forever.
    //
    while(1)
    {
        xSemaphoreTake(miSemaforo2,portMAX_DELAY);
        girar_robot(-90);

       //UARTprintf("He puesto botton drecha ye mandado mensaje\n");
    }
}


static portTASK_FUNCTION(BarraTask,pvParameters)
{
    //xSemaphoreTake(BarraSemaphore,portMAX_DELAY);
    TickType_t ui32LastTime;
    //
    // Loop forever.
    //
    while(1)
    {
        xSemaphoreTake(BarraSemaphore,portMAX_DELAY);
        ui32LastTime = xTaskGetTickCount();
        PWM3Set(1);
        vTaskDelayUntil(&ui32LastTime, 2000);
        PWM3Set(0);

    }
}



static portTASK_FUNCTION(sueloTask,pvParameters)
{
    //xSemaphoreTake(FrontEdgeSemaphore,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
        xSemaphoreTake(FrontEdgeSemaphore,portMAX_DELAY);
        UARTprintf("detectado! \n" );
        //GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);

    }
}


//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************

int main(void)
{
    Setup_Hardware();
    // kp = 15 , ki = 0 , Kd = 0
    PID_Init(&pidA, 15.0f, 0.0f, 0.0f);
    PID_Init(&pidB, 15.0f, 0.0f, 0.0f);
    PWMInit();
    configADC_IniciaADC();

	/********************************      Creacion de tareas *********************/

	//Tarea del interprete de comandos (commands.c)
    if (initCommandLine(COMMAND_TASK_STACK,COMMAND_TASK_PRIORITY) != pdTRUE)
    {
        while(1);
    }

	if((xTaskCreate(ADCTask, (portCHAR *)"ADC", ADC_TASK_STACK,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(Switch1Task,(portCHAR *) "Sw1",SW_TASK_STACK_SIZE, NULL,SW_TASK_PRIO, NULL) != pdTRUE))
    {
        while(1);
    }
    if((xTaskCreate(Switch2Task,(portCHAR *) "Sw2",SW_TASK_STACK_SIZE, NULL,SW_TASK_PRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(BarraTask,(portCHAR *) "BarraTask",SW_TASK_STACK_SIZE, NULL,SW_TASK_PRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(sueloTask,(portCHAR *) "sueloTask",SW_TASK_STACK_SIZE, NULL,SW_TASK_PRIO, NULL) != pdTRUE))
    {
        while(1);
    }

	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

// Rutinas de interrupcion
void GPIOFIntHandler(void){
    //Lee el estado del puerto (activos a nivel bajo)

    int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS );
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    if (!(i32PinStatus & LEFT_BUTTON))
    {
        xSemaphoreGiveFromISR(miSemaforo,&xHigherPriorityTaskWoken);
    }

    if (!(i32PinStatus & RIGHT_BUTTON))
    {
        xSemaphoreGiveFromISR(miSemaforo2,&xHigherPriorityTaskWoken);

    }

    MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


 //Interrupt handler for GPIO Port A (PA3 and PA4)
void encoderInterruptHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check which pin triggered the interrupt and give the corresponding semaphore
    if (GPIOIntStatus(GPIO_PORTA_BASE, true) & GPIO_PIN_2) {
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
        xSemaphoreGiveFromISR(encoderSemaphoreA, &xHigherPriorityTaskWoken);

    }

    if (GPIOIntStatus(GPIO_PORTA_BASE, true) & GPIO_PIN_3) {
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
        xSemaphoreGiveFromISR(encoderSemaphoreB, &xHigherPriorityTaskWoken);

    }

    if (GPIOIntStatus(GPIO_PORTA_BASE, true) & GPIO_PIN_5) {
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
        xSemaphoreGiveFromISR(BarraSemaphore, &xHigherPriorityTaskWoken);

    }

    if (GPIOIntStatus(GPIO_PORTA_BASE, true) & GPIO_PIN_7) {
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
        xSemaphoreGiveFromISR(FrontEdgeSemaphore, &xHigherPriorityTaskWoken);

        GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_7);

        TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000 * 500);
        TimerEnable(TIMER0_BASE, TIMER_A);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void Timer0Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // 清除中断标志
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);
    // 检查传感器引脚状态是否稳定
//    if (GPIOIntStatus(GPIO_PORTA_BASE, true) & GPIO_PIN_7) {
//        // 确认传感器状态稳定，执行操作
//
//       // xSemaphoreGiveFromISR(FrontEdgeSemaphore, &xHigherPriorityTaskWoken);
//    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

