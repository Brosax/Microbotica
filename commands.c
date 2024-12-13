//*****************************************************************************
//
// commands.c - FreeRTOS porting example on CCS4
//
// Este fichero contiene errores que seran explicados en clase
//
//*****************************************************************************


#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard TIVA includes */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
//#include "inc/PWMLib.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

/* Other TIVA includes */
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "drivers/rgb.h"

#include <driver_microbot/PWMLib.h>
#include <driver_microbot/configADC.h>
#include <driver_microbot/pid.h>

int VelocidadF21 = 75;
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
    } Cmuestras;
// ==============================================================================
// The CPU usage in percent, in 16.16 fixed point format.
// ==============================================================================
extern uint32_t g_ui32CPUUsage;
extern PIDController pidA, pidB;

// ==============================================================================
// Implementa el comando cpu que muestra el uso de CPU
// ==============================================================================
static int  Cmd_cpu(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("ARM Cortex-M4F %u MHz - ",SysCtlClockGet() / 1000000);
    UARTprintf("%2u%% de uso\r\n", (g_ui32CPUUsage+32768) >> 16);

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando free, que muestra cuanta memoria "heap" le queda al FreeRTOS
// ==============================================================================
static int Cmd_free(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("%d bytes libres\r\n", xPortGetFreeHeapSize());

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando task. S锟絣o es posible si la opci锟絥 configUSE_TRACE_FACILITY de FreeRTOS est锟� habilitada
// ==============================================================================
#if ( configUSE_TRACE_FACILITY == 1 )

extern char *__stack;
static  int Cmd_tasks(int argc, char *argv[])
{
	char*	pcBuffer;
	uint8_t*	pi8Stack;
	portBASE_TYPE	x;
	
	pcBuffer = pvPortMalloc(1024);
	vTaskList(pcBuffer);
	UARTprintf("\t\t\t\tUnused\r\nTaskName\tStatus\tPri\tStack\tTask ID\r\n");
	UARTprintf("=======================================================\r\n");
	UARTprintf("%s", pcBuffer);
	
	// Calculate kernel stack usage
	x = 0;
	pi8Stack = (uint8_t *) &__stack;
	while (*pi8Stack++ == 0xA5)
	{
		x++;	//Esto s锟絣o funciona si hemos rellenado la pila del sistema con 0xA5 en el arranque
	}
	sprintf((char *) pcBuffer, "%%%us", configMAX_TASK_NAME_LEN);
	sprintf((char *) &pcBuffer[10], (const char *) pcBuffer, "kernel");
	UARTprintf("%s\t-\t*%u\t%u\t-\r\n", &pcBuffer[10], configKERNEL_INTERRUPT_PRIORITY, x/sizeof(portBASE_TYPE));
	vPortFree(pcBuffer);
	return 0;
}

#endif /* configUSE_TRACE_FACILITY */

#if configGENERATE_RUN_TIME_STATS
// ==============================================================================
// Implementa el comando "Stats"
// ==============================================================================
static Cmd_stats(int argc, char *argv[])
{
	char*	pBuffer;

	pBuffer = pvPortMalloc(1024);
	if (pBuffer)
	{
		vTaskGetRunTimeStats(pBuffer); //Es un poco inseguro, pero por ahora nos vale...
		UARTprintf("TaskName\tCycles\t\tPercent\r\n");
		UARTprintf("===============================================\r\r\n");
		UARTprintf("%s", pBuffer);
		vPortFree(pBuffer);
	}
	return 0;
}
#endif

// ==============================================================================
// Implementa el comando help
// ==============================================================================
static int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("Comandos disponibles\r\n");
    UARTprintf("------------------\r\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}


// ==============================================================================
// Implementa el comando "Active PWM"
// ==============================================================================
static int Cmd_AcMo(int argc, char *argv[])
{
    float arrayPWM[2];

    if (argc != 3)
    {
        //Si los par锟絤etros no son suficientes, muestro la ayuda
        UARTprintf(" PWM [PF2] [PF3]\r\n");
    }
    else
    {
        arrayPWM[0]=strtof(argv[1], NULL);
        arrayPWM[1]=strtof(argv[2], NULL);

        if ((arrayPWM[0] > 100)||(arrayPWM[1] > 100)|| (arrayPWM[0] < -100)||(arrayPWM[1] < -100))
        {

            UARTprintf(" out of range \r\n");
        }
        else{
            //activatePWM(arrayPWM[0],arrayPWM[1]);
            PWM1Set(arrayPWM[0]);
            PWM2Set(arrayPWM[1]);
        }
    }

    return 0;
}
static int Cmd_X(int argc, char *argv[])
{
    uint32_t arrayPWM[2];

    if (argc != 1)
    {
        //Si los par锟絤etros no son suficientes, muestro la ayuda
        UARTprintf(" X\r\n");
    }
    else
    {
        arrayPWM[0]=strtoul(argv[1], NULL, 10);
        arrayPWM[1]=strtoul(argv[2], NULL, 10);
        if (VelocidadF21 > 74 && VelocidadF21 < 101){
            if(!(VelocidadF21 == 100))
                    VelocidadF21 = VelocidadF21 + 5;
                    //activatePWM(VelocidadF21,VelocidadF21);
                    PWMSetBoth(VelocidadF21,VelocidadF21);
                }


//        if ((arrayPWM[0] > 100)||(arrayPWM[1] > 100)|| (arrayPWM[0] < 50)||(arrayPWM[1] < 50))
//        {
//
//            UARTprintf(" out of range \r\n");
//        }
//        else{
//            activatePWM(arrayPWM[0],arrayPWM[1]);
//        }
    }

    return 0;
}

static int Cmd_Y(int argc, char *argv[])
{
    uint32_t arrayPWM[2];

    if (argc != 1)
    {
        //Si los par锟絤etros no son suficientes, muestro la ayuda
        UARTprintf(" Y\r\n");
    }
    else
    {
        arrayPWM[0]=strtoul(argv[1], NULL, 10);
        arrayPWM[1]=strtoul(argv[2], NULL, 10);
        if (VelocidadF21 > 74 && VelocidadF21 < 101){
             if(!(VelocidadF21 == 75))
                 VelocidadF21 = VelocidadF21 - 5;
                 PWMSetBoth(VelocidadF21,VelocidadF21);
         }


//        if ((arrayPWM[0] > 100)||(arrayPWM[1] > 100)|| (arrayPWM[0] < 50)||(arrayPWM[1] < 50))
//        {
//
//            UARTprintf(" out of range \r\n");
//        }
//        else{
//            activatePWM(arrayPWM[0],arrayPWM[1]);
//        }
    }

    return 0;
}


//static int Cmd_adc(int argc, char *argv[])
//{
//    MuestrasADCsensor muestras;
//    configADC_DisparaADC();
//
//        //Si los par锟絤etros no son suficientes, muestro la ayuda
//        UARTprintf(" ADC\r\n");
//        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
//        UARTprintf(" ADC %d\n",muestras.chan2);
//
//
//
//
//    return 0;
//}

// ==============================================================================
// Implementa el comando "Desativate PWM"
// ==============================================================================
static int Cmd_DcMo(int argc, char *argv[])
{
    uint32_t arrayPWM[3];

    if (argc != 3)
    {
        //Si los par锟絤etros no son suficientes, muestro la ayuda
        UARTprintf(" PWM [PF2] [PF3]\r\n");
    }
    else
    {
        arrayPWM[0]=strtoul(argv[1], NULL, 10);
        arrayPWM[1]=strtoul(argv[2], NULL, 10);

        if ((arrayPWM[0] > 100)||(arrayPWM[1] > 100))
        {

            UARTprintf(" out of range \r\n");
        }
        else{
            //activatePWM(arrayPWM[0],arrayPWM[1]);
            PWMSetBoth(arrayPWM[0],arrayPWM[1]);
        }
    }

    return 0;
}

// ==============================================================================
// Implementa el comando "MOVE"
// ==============================================================================

static int Cmd_Move(int argc, char *argv[])
{
    float C;

    if (argc != 2)
    {
        //Si los par锟絤etros no son suficientes, muestro la ayuda
        UARTprintf(" MOVE [Distancia en cm] \r\n");
    }
    else
    {
        C=strtof(argv[1], NULL);
        mover_robot(&C);

    }

    return 0;
}
// ==============================================================================
// Implementa el comando "MOVE"
// ==============================================================================

static int Cmd_Gira(int argc, char *argv[])
{
    float g;

    if (argc != 2)
    {
        //Si los par锟絤etros no son suficientes, muestro la ayuda
        UARTprintf(" GIRA [GRADOS] \r\n");
    }
    else
    {
        g=strtof(argv[1], NULL);
        girar_robot(&g);

    }

    return 0;
}


// ==============================================================================
// Implementa el comando "PID"
// ==============================================================================

static int Cmd_PID(int argc, char *argv[])
{
    float ArrayPid[3];

    if (argc != 4)
    {
        //Si los parametros no son suficientes, muestro la ayuda
        UARTprintf(" PID [Kp] [Ki] [Kd]\r\n");
    }
    else
    {
        ArrayPid[0]=strtof(argv[1], NULL);
        ArrayPid[1]=strtof(argv[2], NULL);
        ArrayPid[2]=strtof(argv[3], NULL);

        PID_Init(&pidA, ArrayPid[0], ArrayPid[1], ArrayPid[2]);
        PID_Init(&pidB, ArrayPid[0], ArrayPid[1], ArrayPid[2]);


    }

    return 0;
}

static int Cmd_FORK(int argc, char *argv[])
{
    uint32_t state;

    if (argc != 2)
    {
        //Si los parametros no son suficientes, muestro la ayuda
        UARTprintf(" Actfork [state]\r\n");
    }
    else
    {
        state=strtoul(argv[1], NULL,10);
        PWM3Set(state);
    }

    return 0;
}


// ==============================================================================
// Tabla con los comandos y su descripcion. Si quiero anadir alguno, debo hacerlo aqui
// ==============================================================================
//Este array tiene que ser global porque es utilizado por la biblioteca cmdline.c para implementar el interprete de comandos
//No es muy elegante, pero es lo que ha implementado Texas Instruments.
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,      "     : Lista de comandos" },
    { "?",        Cmd_help,      "        : lo mismo que help" },
    { "cpu",      Cmd_cpu,       "      : Muestra el uso de  CPU " },
    { "free",     Cmd_free,      "     : Muestra la memoria libre" },
    { "ActMot",   Cmd_AcMo,     "    : Activate PWM for Motor" },
    { "DecMot",   Cmd_DcMo,     "    : Desvtivate PWM" },
    { "X",        Cmd_X,     "    : Subir Vlocidad Motor" },
    { "Y",        Cmd_Y,     "    : Bajar Velocidad Motor" },
    { "MOVE",     Cmd_Move,     "    : Mover robot rectos hacia alante" },
    { "GIRA",     Cmd_Gira,     "    : Girar robot un angulo" },
    { "PID",      Cmd_PID,     "    : configura parametros del PID" },
    { "Actfork",  Cmd_FORK,     "    : Activar fork " },

    #if ( configUSE_TRACE_FACILITY == 1 )
	{ "tasks",    Cmd_tasks,     "    : Muestra informacion de las tareas" },
#endif
#if (configGENERATE_RUN_TIME_STATS)
	{ "stats",    Cmd_stats,      "     : Muestra estadisticas de las tareas" },
#endif
    { 0, 0, 0 }
};

// ==============================================================================
// Tarea UARTTask.  Espera la llegada de comandos por el puerto serie y los ejecuta al recibirlos...
// ==============================================================================
static void vCommandTask( void *pvParameters )
{
    char    pcCmdBuf[64];
    int32_t i32Status;
	
    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\r\n\r\nWelcome to the TIVA FreeRTOS Demo!\r\n");
	UARTprintf("\r\n\r\n FreeRTOS %s \r\n",
		tskKERNEL_VERSION_NUMBER);
	UARTprintf("\r\n Teclee ? para ver la ayuda \r\n");
	UARTprintf("> ");    

	/* Loop forever */
	while (1)
	{

		/* Read data from the UART and process the command line */
		UARTgets(pcCmdBuf, sizeof(pcCmdBuf));
		if (strlen(pcCmdBuf) == 0)
		{
			UARTprintf("> ");
			continue;
		}

		//
		// Pass the line from the user to the command processor.  It will be
		// parsed and valid commands executed.
		//
		i32Status = CmdLineProcess(pcCmdBuf);

		//
		// Handle the case of bad command.
		//
		if(i32Status == CMDLINE_BAD_CMD)
		{
			UARTprintf("Comando erroneo!\r\n");	//No pongo acentos adrede
		}

		//
		// Handle the case of too many arguments.
		//
		else if(i32Status == CMDLINE_TOO_MANY_ARGS)
		{
			UARTprintf("El interprete de comandos no admite tantos parametros\r\n");	//El maximo, CMDLINE_MAX_ARGS, esta definido en cmdline.c
		}

		//
		// Otherwise the command was executed.  Print the error code if one was
		// returned.
		//
		else if(i32Status != 0)
		{
			UARTprintf("El comando devolvio el error %d\r\n",i32Status);
		}

		UARTprintf("> ");

	}
}




//
// Create la tarea que gestiona los comandos (definida en el fichero commands.c)
//
BaseType_t initCommandLine(uint16_t stack_size,uint8_t prioriry )
{

    // Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
    //se usa para mandar y recibir mensajes y comandos por el puerto serie
    // Mediante un programa terminal como gtkterm, putty, cutecom, etc...
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Esta funcion habilita la interrupcion de la UART y le da la prioridad adecuada si esta activado el soporte para FreeRTOS
    UARTStdioConfig(0,115200,SysCtlClockGet());
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);   //La UART tiene que seguir funcionando aunque el micro esta dormido

    return xTaskCreate(vCommandTask, (signed portCHAR *)"UartComm", stack_size,NULL,prioriry, NULL);
}
