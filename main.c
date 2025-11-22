////////////////////////////////////////////////////////////////////////////////////

/// File: main.c
/// Created by: Grace Harding, Ben Kensington
/// Lab 8
/// Bare-metal approach to follow-the-line robot

////////////////////////////////////////////////////////////////////////////////////
/// INCLUDES AND DEFINES

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

#include "Pmod_Dual_MAXSONAR.h"
#include "PWM.h"
#include "Pmod_DHB1.h"

/// Note on Pmod connections:
// JA = Pmod LS1 -> IR sensors are on S1 and S4
// JB = Pmod MAXSONAR -> sonar0 is on channel 0 and sonar1 on channel 1
// JD = Pmod DHB1

/// Vivado Hardware Defines
//#define AXI_GPIO_0_BASEADDR       0x40000000
//#define AXI_GPIO_1_BASEADDR       0x40010000
//#define AXI_GPIO_2_BASEADDR       0x40020000
#define LS1_BASEADDR              0x40020000 //axi_gpio_2
#define IR_L_SENSOR               0x1
#define IR_R_SENSOR               0x2
#define DHB1_GPIO_BASEADDR        0x44A00000
#define DHB1_MOTOR_FB_BASEADDR    0x44A10000
#define DHB1_PWM_BASEADDR         0x44A20000
#define Dual_MAXSONAR_0_BASEADDR  0x44A30000
#define XPAR_TMRCTR_0_DEVICE_ID   0x41C00000
#define XPAR_INTC_0_DEVICE_ID     0x41200000

/// Frequency defines
#define CLK_FREQ            81247000
#define DHB1_PWM_PERIOD     0x2   // 2 ms period

/// Timer and load value defines
#define LOAD_VALUE          40624 // ~0.5ms period (0.5000012 closer to actual)
#define TIMER_PERIOD_US     500   // Timer period in microseconds (500 us = 0.5 ms)

/// Interrupt and timer ID defines
#define TMRCTR_DEVICE_ID    XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID XPAR_FABRIC_XTMRCTR_0_INTR
#define INTC_DEVICE_ID      XPAR_INTC_0_DEVICE_ID

/// Motor defines
#define MOTOR_FORWARD       0 // this is just a guess. if forward != 0, then its 1
#define MOTOR_REVERSE       1 // this is just a guess. if reverse != 1, then its 0
#define MOTOR_MAX_SPEED     100
#define MOTOR_MIN_SPEED     0

////////////////////////////////////////////////////////////////////////////////////
/// FUNCTION PROTOTYPES

/// ISR and delay functions
//void timer_ISR(void *CallBackRef, u8 TmrCtrNumber);
void delay(int ms);

/// Helper functions
int platform_init();
void executionFailed();
void setupTasks();

/// Tick functions
void Motor_Tick();

/// Tasks
void taskSupervisor(void *data);
void taskSonar(void *data);

////////////////////////////////////////////////////////////////////////////////////
/// Enums, structs and global variables

// Enumerated type for the tasks
typedef enum
{
  TASK_SUPERVISOR,
  TASK_SONAR,
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

// Timer counter -- multiply by TIMER_PERIOD_US to get time elapsed since system boot in microseconds
uint32_t TimerCounter;

// State variable
state currentState;

// Motor Controller, Global Speed Variables
PmodDHB1* MotorController;
u8 LeftMotorSpeed;          // This variable's value will be constantly applied to the left motor
u8 RightMotorSpeed;         // This variable's value will be constantly applied to the right motor

// Hardware instances
XIntc InterruptController;  // Instance of the Interrupt Controller
XTmrCtr Timer;              // Instance of the Timer
XGpio lightGpio;            // Instance of the AXI_GPIO_2

////////////////////////////////////////////////////////////////////////////////////
/// ISRs and Delay functions

// void timer_ISR(void *CallBackRef, u8 TmrCtrNumber)
// {
//   // Increment system time
//   TimerCounter++; // 1 unit = 0.5ms

//   // Get instance of the timer linked to the interrupt
//   XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

//   // Check if the timer counter has expired
//   if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber))
//     // Queue our supervisor to run
//     queue[TASK_SUPERVISOR]->taskReady = TRUE;
// }

void delay(int ms){
  static count = ms;

  // TODO: implement delay based on 
}

////////////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS

void testLightSensors() {
  // Get reference infrared sensor registers
  volatile u32 *InfraredData =        (u32 *)LS1_BASEADDR + XGPIO_DATA_OFFSET;
  volatile u32 *InfraredTristateReg = (u32 *)LS1_BASEADDR + XGPIO_TRI_OFFSET;

  *InfraredTristateReg = 0xF;

  while (1)
	{
		if (*InfraredData & IR_L_SENSOR) // left sensor touched the reflective tape
			xil_printf("left!\r");

		if (*InfraredData & IR_R_SENSOR) // right sensor touched the reflective tape
			xil_printf("right!\r");

		xil_printf("0x%08x\r", *InfraredData);
	}
}

void testSonar() {
  // Initialize sonar instances
  PMOD_DUAL_MAXSONAR sonar0 = {Dual_MAXSONAR_0_BASEADDR + MAXSONAR_CHANNEL_0_OFFSET, CLK_FREQ, 0};
  PMOD_DUAL_MAXSONAR sonar1 = {Dual_MAXSONAR_0_BASEADDR + MAXSONAR_CHANNEL_1_OFFSET, CLK_FREQ, 0};

  // Start sonar instances
  MAXSONAR_begin(&sonar0, Dual_MAXSONAR_0_BASEADDR, CLK_FREQ);
  MAXSONAR_begin(&sonar1, Dual_MAXSONAR_0_BASEADDR, CLK_FREQ);

  while (1)
  {
    // NOTE:
    // The lab8 doc on HThreads notes that getDistance returns the value in inches
    // However, the original lab8 example that uses getDistance implies its in cm
    // we need to test and see what it actually is.

    u16 distance0 = MAXSONAR_getDistance(&sonar0, MAXSONAR_1);
    u16 distance1 = MAXSONAR_getDistance(&sonar1, MAXSONAR_2);
    xil_printf("Sonar0: %d cm, Sonar1: %d cm\r", distance0, distance1);
  }
}

void testMotor() {
    DHB1_begin(MotorController, DHB1_GPIO_BASEADDR, DHB1_PWM_BASEADDR,
                CLK_FREQ, DHB1_PWM_PERIOD);

    DHB1_motorEnable(MotorController);
    DHB1_setDirs(MotorController, MOTOR_FORWARD, MOTOR_FORWARD);
    
    DHB1_setMotorSpeeds(MotorController, MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    DHB1_motorDisable(MotorController);
}

////////////////////////////////////////////////////////////////////////////////////
/// ROBOT FUNCTIONS

/// Update motor speeds based on global speed variables
void Motor_Tick() {

  // Do PID Calculation shtuff? (if we're going with PID)

  // Apply current speed vars to motor
  DHB1_setMotorSpeeds(MotorController, LeftMotorSpeed, RightMotorSpeed);
}


////////////////////////////////////////////////////////////////////////////////////
/// HARDWARE INITIALIZATION

/// Timer, Interrupt Controller and lightGPIO Initialization
int platform_init()
{
  int status = XST_FAILURE;

  // Initialize the GPIO instances
  status = XGpio_Initialize(&lightGpio, LS1_BASEADDR);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize GPIO_2! Execution stopped.\n");
    executionFailed();
  }

  // Set GPIO_0 CHANNEL 2 as input
  XGpio_SetDataDirection(&lightGpio, 0x2, 0xF);

  // Set GPIO_1 CHANNEL 1 as output
  XGpio_SetDataDirection(&lightGpio, 0x1, 0xF);

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

/// Function that is called when hardware couldn't be initialized (via platform_init())
void executionFailed()
{
  xil_printf("Execution Failed");
  // trap the program in an infinite loop
  while (1);
}

////////////////////////////////////////////////////////////////////////////////////
/// MISC FUNCTIONS

/// Setup task queue
void setupTasks() {
  // Task 0: taskSupervisor
  queue[TASK_SUPERVISOR] = malloc(sizeof(TCB_t));
  queue[TASK_SUPERVISOR]->taskPtr = taskSupervisor;
  queue[TASK_SUPERVISOR]->taskDataPtr = NULL;
  queue[TASK_SUPERVISOR]->taskReady = FALSE;

  // Task 1: taskSonar
  queue[TASK_SONAR] = malloc(sizeof(TCB_t));
  queue[TASK_SONAR]->taskPtr = taskSonar;
  queue[TASK_SONAR]->taskDataPtr = NULL;
  queue[TASK_SONAR]->taskReady = TRUE;
}

////////////////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTION

int main(int argc, char const *argv[])
{
  // Initialize timer counter, motor speeds, and state
  TimerCounter =    0;
  LeftMotorSpeed =  MOTOR_MIN_SPEED;
  RightMotorSpeed = MOTOR_MIN_SPEED;
  currentState =    STATE_IDLE;

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
/// TASK IMPLEMENTATIONS

/// Supervisor task
void taskSupervisor(void *data) {
  switch(currentState)
  {
    case STATE_IDLE:
      queue[TASK_SONAR]->taskReady = TRUE; // Start sonar task
      break;

    default:
      // Invalid state go idle silly
      currentState = STATE_IDLE;
      break;
  }
}

/// Sonar measurement task
void taskSonar(void *data) {
  testSonar();
}