////////////////////////////////////////////////////////////////////////////////////

/// File: main.c
/// Created by: Grace Harding, Ben Kensington
/// Lab 8
/// Bare-metal approach to follow-the-line robot

////////////////////////////////////////////////////////////////////////////////////
/// Includes and Defines

#include <xil_printf.h>
#include <xparameters.h>
#include <xtmrctr.h>
#include <xtmrctr_l.h>
#include <xil_exception.h>
#include <xil_printf.h>
#include <xintc.h>
#include <xgpio.h>
#include <stdlib.h>
#include "xil_cache.h"
#include "xgpio.h"
#include "xgpio_l.h"

#include "PmodMAXSONAR.h"
#include "PWM.h"
#include "PmodDHB1.h"

/// Note on Pmod connections:
/// JA = Pmod LS1 -- note, IR sensors are on S1 and S4
/// JB = Pmod MAXSONAR -- note, sonar0 is on channel 0 and sonar1 on channel 1
/// JD = Pmod DHB1

// Infrared Sensor Defines
#define LS1_BASEADDR              XPAR_AXI_GPIO_PMOD_LS1_BASEADDR // Note: this may be inaccurate!
#define IR_L_SENSOR               0x1
#define IR_R_SENSOR               0x2

// Sonar Sensor Defines -- Note: may be inaccurate!
#define PMOD_SONAR0_BASEADDR      XPAR_PMOD_DUAL_MAXSONAR_0_SONAR0_BASEADDR
#define PMOD_SONAR1_BASEADDR      XPAR_PMOD_DUAL_MAXSONAR_0_SONAR1_BASEADDR

// Vivado Hardware Defines
#define AXI_GPIO_0_BASEADDR       0x40000000
#define AXI_GPIO_1_BASEADDR       0x40010000
#define AXI_GPIO_2_BASEADDR       0x40020000
#define DHB1_GPIO_BASEADDR        0x44A00000
#define DHB1_MOTOR_FB_BASEADDR    0x44A10000
#define DHB1_PWM_BASEADDR         0x44A20000
#define Dual_MAXSONAR_0_BASEADDR  0x44A30000

// PWM Defines
#define PWM_PERIOD 0x00029000 // 2ms
#define PWM_DUTY 0x00014800   // 50% duty cycle
#define PWM_M1 0
#define PWM_M2 1

// Clock frequency define (may be inaccurate! I don't think 8124700 is exactly the Arty's clock)
#ifdef __MICROBLAZE__
#define CLK_FREQ XPAR_CPU_M_AXI_DP_FREQ_HZ
#else
#define CLK_FREQ 81247000 // FCLK0 frequency not found in xparameters.h
#endif

// Timer and load value defines
#define LOAD_VALUE 40624 // ~0.5ms period (0.5000012 closer to actual)
#define TIMER_PERIOD_US 500 // Timer period in microseconds (500 us = 0.5 ms)

// Interrupt and timer ID defines
#define TMRCTR_DEVICE_ID XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID XPAR_INTC_0_TMRCTR_0_VEC_ID
#define INTC_DEVICE_ID XPAR_INTC_0_DEVICE_ID

////////////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS

void testInfrared() {
  // Get reference infrared sensor registers
  volatile u32 *InfraredData = (u32 *)LS1_BASEADDR + XGPIO_DATA_OFFSET;
	volatile u32 *InfraredTristateReg = (u32 *)LS1_BASEADDR + XGPIO_TRI_OFFSET;

	*InfraredTristateReg = 0xF;

  while (1)
	{
		if (*InfraredData & IR_L_SENSOR)
			xil_printf("left!\r");

		if (*InfraredData & IR_R_SENSOR)
			xil_printf("right!\r");

		xil_printf("0x%08x\r", *InfraredData);
	}
}

void testSonar() {
  // Initialize sonar instances
  PmodMAXSONAR sonar0;
  PmodMAXSONAR sonar1;
  MAXSONAR_Init(&sonar0, PMOD_SONAR0_BASEADDR);
  MAXSONAR_Init(&sonar1, PMOD_SONAR1_BASEADDR);

  while (1)
  {
    u16 distance0 = MAXSONAR_GetDistance(&sonar0);
    u16 distance1 = MAXSONAR_GetDistance(&sonar1);
    xil_printf("Sonar0: %d cm, Sonar1: %d cm\r", distance0, distance1);
  }
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
/// Function prototypes

// Timer Interrupt Service Routine
void timer_ISR(void *CallBackRef, u8 TmrCtrNumber);
// Helper functions
int platform_init();
void TmrCtrDisableIntr(XIntc *IntcInstancePtr, u16 IntrId);
void executionFailed();
void setupTasks();

// Tasks
void taskSupervisor(void *data);

////////////////////////////////////////////////////////////////////////////////////
/// Enums, structs and global variables

// Enumerated type for the tasks
typedef enum
{
  TASK_SUPERVISOR,
  MAX_TASKS
} task_t;

// Enumerated type for states -- i.e. (move forward, turn left, turn right, etc)
typedef enum
{
  STATE_IDLE,
  MAX_STATES
} state;

// Task Control Block (TCB) structure
typedef struct
{
  void (*taskPtr)(void *);
  void *taskDataPtr;
  u8 taskReady;
} TCB_t;

// Task queue
TCB_t *queue[MAX_TASKS];

// Timer counter -- multiply by TIMER_PERIOD_US to get time in microseconds
volatile static uint32_t MasterTimerCounter;

// State variable
volatile static state currentState;

// Hardware instances
XIntc InterruptController;  // Instance of the Interrupt Controller
XTmrCtr Timer;              // Instance of the Timer
XGpio btnGpio;              // Instance of the AXI_GPIO_0
XGpio ledGpio;              // Instance of the AXI_GPIO_1

void timer_ISR(void *CallBackRef, u8 TmrCtrNumber)
{
  // Increment system time
  MasterTimerCounter++; // 1 unit = 0.5ms

  // Get instance of the timer linked to the interrupt
  XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

  // Check if the timer counter has expired
  if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber))
    // Queue our supervisor to run
    queue[TASK_SUPERVISOR]->taskReady = TRUE;
}

////////////////////////////////////////////////////////////////////////////////////
/// "Finished" functions (from Lab 6)
int platform_init()
{
  int status = XST_FAILURE;

  // Initialize the GPIO instances
  status = XGpio_Initialize(&btnGpio, XPAR_GPIO_0_DEVICE_ID);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize GPIO_0! Execution stopped.\n");
    executionFailed();
  }

  status = XGpio_Initialize(&ledGpio, XPAR_GPIO_1_DEVICE_ID);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize GPIO_1! Execution stopped.\n");
    executionFailed();
  }

  // Set GPIO_0 CHANNEL 2 as input
  XGpio_SetDataDirection(&btnGpio, CHANNEL_2, 0xF);

  // Set GPIO_1 CHANNEL 1 as output
  XGpio_SetDataDirection(&ledGpio, CHANNEL_1, 0x0);

  // Initialize the timer counter instance
  status = XTmrCtr_Initialize(&Timer, TMRCTR_DEVICE_ID);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize the timer! Execution stopped.\n");
    executionFailed();
  }

  // Verifies the specified timer is setup correctly in hardware/software
  status = XTmrCtr_SelfTest(&Timer, XTC_TIMER_0);
  if (status != XST_SUCCESS)
  {
    xil_printf("Testing timer operation failed! Execution stopped.\n");
    executionFailed();
  }

  // Initialize the interrupt controller instance
  status = XIntc_Initialize(&InterruptController, INTC_DEVICE_ID);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize the interrupt controller! Execution stopped.\n");
    executionFailed();
  }

  // Connect a timer handler that will be called when an interrupt occurs
  status = XIntc_Connect(&InterruptController,
                         TMRCTR_INTERRUPT_ID,
                         (XInterruptHandler)XTmrCtr_InterruptHandler,
                         (void *)&Timer);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to connect timer handler! Execution stopped.\n");
    executionFailed();
  }

  // Start the interrupt controller
  status = XIntc_Start(&InterruptController, XIN_REAL_MODE);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to start interrupt controller! Execution stopped.\n");
    executionFailed();
  }

  // Enable interrupts and the exception table
  XIntc_Enable(&InterruptController, TMRCTR_INTERRUPT_ID);
  Xil_ExceptionInit();

  // Register the interrupt controller handler with the exception table.
  Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                               (Xil_ExceptionHandler)XIntc_InterruptHandler,
                               &InterruptController);

  // Enable exceptions
  Xil_ExceptionEnable();

  // Set the handler (function pointer) that we want to execute when the
  // interrupt occurs
  XTmrCtr_SetHandler(&Timer, timer_ISR, &Timer);

  // Set our timer options (setting TCSR register indirectly through Xil API)
  u32 timerConfig = XTC_INT_MODE_OPTION |
                    XTC_DOWN_COUNT_OPTION |
                    XTC_AUTO_RELOAD_OPTION;
  XTmrCtr_SetOptions(&Timer, XTC_TIMER_0, timerConfig);

  // Set what value the timer should reset/init to (setting TLR indirectly)
  XTmrCtr_SetResetValue(&Timer, XTC_TIMER_0, LOAD_VALUE);

  // Start the timer
  XTmrCtr_Start(&Timer, XTC_TIMER_0);

  return XST_SUCCESS;
}

void executionFailed()
{
  u32 *rgbLedsData = (u32 *)(XPAR_GPIO_1_BASEADDR);
  *rgbLedsData = RGB_RED; // display all red LEDs if fail state occurs

  // trap the program in an infinite loop
  while (1);
}

// Optional demonstration on how to disable interrupt
void TmrCtrDisableIntr(XIntc *IntcInstancePtr, u16 IntrId)
{
  // Disable the interrupt for the timer counter
  XIntc_Disable(IntcInstancePtr, IntrId);

  return;
}

////////////////////////////////////////////////////////////////////////////////////
/// Misc functions
void setupTasks() {
  // Task 0: taskSupervisor
  queue[TASK_CHOOSE] = malloc(sizeof(TCB_t));
  queue[TASK_CHOOSE]->taskPtr = taskSupervisor;
  queue[TASK_CHOOSE]->taskDataPtr = NULL;
  queue[TASK_CHOOSE]->taskReady = FALSE;
}

////////////////////////////////////////////////////////////////////////////////////
/// Main function

int main(int argc, char const *argv[])
{

  // Init timer counter and other variables
  MasterTimerCounter = 0;
  currentState = STATE_IDLE;

  // Setup the GPIO, Interrupt Controller and Timer
  int status = XST_FAILURE;
  status = platform_init();
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize the platform! Execution stopped.\n");
    executionFailed();
  }

  // Initialize task queue and all of its tasks
  setupTasks();

  // Main loop
  while (1)
  {
    // Iterate through task queue, execute 'ready' tasks
    for (int i = 0; i < MAX_TASKS; i++)
    {
      // Execute tasks that are only 'ready'
      if (queue[i]->taskReady)
      {
        // Execute the task
        (*(queue[i]->taskPtr))(queue[i]->taskDataPtr);

        // Reset the task ready flag
        queue[i]->taskReady = 0;
      }
    }
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// Task implementations
void taskSupervisor(void *data)
{
  switch(currentState)
  {
    case STATE_IDLE:
      // Add more states!
      break;

    default:
      // Invalid state go idle silly
      currentState = STATE_IDLE;
      break;
  }
}