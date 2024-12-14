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
#include "timers.h"
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
//#define wheelTASKPRIO (tskIDLE_PRIORITY+1)          // Prioridad para la tarea wheelTASK
//#define wheelTASKSTACKSIZE (256)                    // Tamanio de pila para la tarea wheelTASK


#define RADIO 3.0f  // Wheel radius in cm
//#define Convertir_Angulo (360.0f / (2.0f * M_PI))
#define Convertir_Angulo 57.29f //360.0f / (2.0f * M_PI)
#define Resolucion 20 //en grado

#define L 10    //Separaci¨®n entre ruedas

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


// 定义软件定时器句柄
TimerHandle_t BarraTimer;
SemaphoreHandle_t miSemaforo,miSemaforo2,miSemaforo3;
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
    ROM_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5, GPIO_DIR_MODE_IN);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5);
    MAP_IntPrioritySet(INT_GPIOA,configMAX_SYSCALL_INTERRUPT_PRIORITY);//para añadir prioridad by HAMED

    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_BOTH_EDGES); // Configure interrupt on both rising and falling edges

    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_RISING_EDGE); // Configure interrupt rising edges ,barra frontal.

    MAP_GPIOIntEnable(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5);
    MAP_IntEnable(INT_GPIOA);




    miSemaforo  = xSemaphoreCreateBinary();
    miSemaforo2 = xSemaphoreCreateBinary();
    miSemaforo3 = xSemaphoreCreateBinary();
    if (miSemaforo == NULL || miSemaforo2 == NULL || miSemaforo3 == NULL) {
           while (1);
        }
    encoderSemaphoreA = xSemaphoreCreateBinary();
    encoderSemaphoreB = xSemaphoreCreateBinary();
    BarraSemaphore = xSemaphoreCreateBinary();

    if (encoderSemaphoreA == NULL || encoderSemaphoreB == NULL || BarraSemaphore == NULL) {
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
    mover_robot(12);
    SysCtlDelay(600000);

    girar_robot(90);
    SysCtlDelay(600000);

    mover_robot(18);
    SysCtlDelay(600000);

    girar_robot(90);
    SysCtlDelay(600000);

    mover_robot(12);
    SysCtlDelay(600000);

    girar_robot(90);
    SysCtlDelay(600000);

    mover_robot(18);
    SysCtlDelay(600000);

    girar_robot(90);

    return 0;
}
//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************


//Funcion de leer sensor de distancia y enciende led
static portTASK_FUNCTION(ADCTask,pvParameters)
{

    MuestrasADCsensor muestras;
  //  MESSAGE_ADC_SAMPLE_PARAMETER parameter;
    double distancia = 1110 ;

    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)

        //UARTprintf("%d %d %d\r\n",muestras.chan1 ,muestras.chan2 , muestras.chan8);

        if( muestras.chan2 < 3420 && muestras.chan2 > 899)
        {
            distancia = -(muestras.chan2 - 3614) / 179.08;
        }

        else if( muestras.chan2 < 900 && muestras.chan2 > 286)
        {
            distancia = -(muestras.chan2 - 1222.6) / 24.853;
        }
        else
        {
            distancia = 111111;
        }




        if (distancia >= 5 && distancia < 10 )
        {
            // cm se enciende el led verde PF3
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00000008);
        }
        else if (distancia >= 10 && distancia < 15 )
        {
                    // cm se enciende el led rojo PF1
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00000002);

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0x00000002);

        }
        else if (distancia >= 15 && distancia <= 20 )
        {
                    //  cm se encienden ambos leds rojo y verde
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 ,0x00000002);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,0x00000008);
        }
        else
        {
            //los leds permanecen apagados
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);
        }

    }
}






//static void Switch1Task(void *pvParameters)
static portTASK_FUNCTION(Switch1Task,pvParameters)
{

    xSemaphoreTake(miSemaforo,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {

         lazocerado();

       xSemaphoreTake(miSemaforo,portMAX_DELAY);
    }
}

static portTASK_FUNCTION(Switch2Task,pvParameters)
{
    xSemaphoreTake(miSemaforo2,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {

       mover_robot(18.22);//eligimos para probar mover
        xSemaphoreTake(miSemaforo2,portMAX_DELAY);
       //UARTprintf("He puesto botton drecha ye mandado mensaje\n");
    }
}

static portTASK_FUNCTION(Switch3Task,pvParameters)
{
    xSemaphoreTake(miSemaforo3,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
//        configADC_DisparaADC(); //Dispara la conversion (por software)
        xSemaphoreTake(miSemaforo3,portMAX_DELAY);
        //UARTprintf("He puesto botton drecha ye mandado mensaje\n");
    }
}

static portTASK_FUNCTION(BarraTask,pvParameters)
{
    xSemaphoreTake(BarraSemaphore,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
        xSemaphoreTake(BarraSemaphore,portMAX_DELAY);
//        PWM3Set(1);
//        TaskDelay(pdMS_TO_TICKS(500));
//        PWM3Set(0);
    }
}


// Función para mapear los valores de un rango a otro
//float map(float value, float in_min, float in_max, float out_min, float out_max) {
//    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}

// Función para convertir los valores del joystick (x, y) en señales para los motores
//void joystickToMotor(float x, float y, int* motor1, int* motor2) {
//    // Normalizar el valor del joystick (x, y) para que est en el rango [-1, 1]
//    float magnitude = sqrt(x * x + y * y);
//    if (magnitude > 1) {
//        x /= magnitude;
//        y /= magnitude;
//    }
//
//    // Convertir el valor de 'x' del joystick para el rango de los motores (50-100)
//    // Para motor1 (gira en un sentido)
//    *motor1 = (int)map(x, -1.0, 1.0, 50, 100);
//
//    // Para motor2 (gira en sentido contrario al motor1)
//    *motor2 = (int)map(x, -1.0, 1.0, 100, 50);
//}


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
    if((xTaskCreate(Switch3Task,(portCHAR *) "Sw3",SW_TASK_STACK_SIZE, NULL,SW_TASK_PRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(BarraTask,(portCHAR *) "BarraTask",SW_TASK_STACK_SIZE, NULL,SW_TASK_PRIO, NULL) != pdTRUE))
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

    int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS | GPIO_PIN_4);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOF);
    int ui8Delay = 0;

    for(ui8Delay = 0; ui8Delay < 16; ui8Delay++)
    {
    }
    if (!(i32PinStatus & LEFT_BUTTON))
    {
        xSemaphoreGiveFromISR(miSemaforo,&xHigherPriorityTaskWoken);
    }

    if (!(i32PinStatus & RIGHT_BUTTON))
    {
        xSemaphoreGiveFromISR(miSemaforo2,&xHigherPriorityTaskWoken);

    }

    if ((i32PinStatus & GPIO_PIN_4))
    {
        xSemaphoreGiveFromISR(miSemaforo3,&xHigherPriorityTaskWoken);

    }
    MAP_IntEnable(INT_GPIOF);
    MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS | GPIO_PIN_4);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}






// 软件定时器回调函数
void BarraTimerCallback(TimerHandle_t xTimer)
{
    // 关闭 PWM 输出
    PWM3Set(0);
}

// 初始化事件驱动的 Barra 控制
void InitBarraEventDriven(void)
{
    // 创建定时器，触发一次后自动停止
    BarraTimer = xTimerCreate(
        "BarraTimer",            // 定时器名称
        pdMS_TO_TICKS(500),      // 定时器周期：500ms
        pdFALSE,                 // 非周期性定时器
        NULL,                    // 定时器标识符（未使用）
        BarraTimerCallback       // 定时器回调函数
    );

    if (BarraTimer == NULL)
    {
        // 错误处理：无法创建定时器
        UARTprintf("Error: Failed to create BarraTimer\n");
        while (1);
    }
}

// 事件触发函数
void TriggerBarraEvent(void)
{
    // 激活 PWM
    PWM3Set(1);

    // 启动定时器，用于关闭 PWM
    if (xTimerStart(BarraTimer, 0) != pdPASS)
    {
        // 错误处理：无法启动定时器
        UARTprintf("Error: Failed to start BarraTimer\n");
    }
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
        TriggerBarraEvent();
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}





