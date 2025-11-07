////////////////////////////////////////////////////////////////////////////////////

/// File: main.c
/// Created by: Grace Harding, Ben Kensington
/// Lab 8

////////////////////////////////////////////////////////////////////////////////////
/// Includes and Defines

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include <projdefs.h>
#include <xil_exception.h>
/* Standard library include. */
#include <stdlib.h>

/// Defines for true/false logic
#define TRUE 1
#define FALSE 0

/// Base Address Defines
#define AXI_GPIO_0_BASE_ADDR 0x40000000 // Button
#define AXI_GPIO_1_BASE_ADDR 0x40010000 // LED
#define AXI_GPIO_SOME_ADDR 0x00000000 // TODO: Add more defines based on Vivado hardware addresses

/// Defines for channel numbers
#define CHANNEL_1 1
#define CHANNEL_2 2

////////////////////////////////////////////////////////////////////////////////////
/// Function Prototypes

/// Task functions
void someTaskHere(void* arg);

/// Helper functions
void initSupervisor();
void SupervisorTask();
void executionFailed();
int transitionState();

////////////////////////////////////////////////////////////////////////////////////
/// State Machine and Variable Initialization

/// State Machine
typedef enum
{
    SOME_STATE_HERE = 0,
    ANOTHER_STATE_HERE
} State;

/// Hardware instances
// XGpio btnGpio;                       // Instance of the AXI_GPIO_0
// XGpio ledGpio;                       // Instance of the AXI_GPIO_1

/// Global variables
// static volatile int someVarHere = 0;

/// Mutexes
// SemaphoreHandle_t state_mutex;
// SemaphoreHandle_t blink_mutex;

/// FreeRTOS Tasks
// TaskHandle_t SupervisorTaskHandle = NULL;

////////////////////////////////////////////////////////////////////////////////////
/// GPIO and etc Initialization

void initGPIO() {
    return;
}

////////////////////////////////////////////////////////////////////////////////////
/// Supervisor Task and Initialization

void SupervisorTask(void *arg) {
    // Infinite loop for supervisor task
    while(1) {

        /// Mutex lock here

        /// Switch/FSM logic here
        // switch(state) { 
        //     case SOME_TASK_HERE:
        //         break;
        // }

        /// Mutex unlock here
    }
}

////////////////////////////////////////////////////////////////////////////////////
/// Main function

int main(void)
{
    /// Initialize directionality for GPIOs/etc...
    // initGPIO();

    /// Initialize supervisor task/etc...
    // initSupervisor();

    /// Start FreeRTOS Kernel
    // vTaskStartScheduler();

    return 0;
}
