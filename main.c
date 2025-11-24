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

#define AXI_GPIO_1_BASE_ADDR 0x40010000
#define AXI_GPIO_0_BASE_ADDR 0x40000000

#define RGB_LEDS_BASE_ADDR (AXI_GPIO_1_BASE_ADDR)
#define SWITCHES_BASE_ADDR (AXI_GPIO_1_BASE_ADDR + 8)

#define RBG_LEDS_REG (unsigned *)(RGB_LEDS_BASE_ADDR)
#define SWITCHES_REG (unsigned *)(SWITCHES_BASE_ADDR)

#define GPIO_DEVICE_ID 0 
#define XPAR_AXI_GPIO_0_INTERRUPTS 0x2001

/// Frequency defines
#define CLK_FREQ              81247000
#define DHB1_PWM_PERIOD       0x2   // 2 ms period

/// Timer and load value defines
#define LOAD_VALUE            40624 // ~0.5ms period (0.5000012 closer to actual)
#define TIMER_PERIOD_US       500   // Timer period in microseconds (500 us = 0.5 ms)

/// Interrupt and timer ID defines
#define TMRCTR_DEVICE_ID      XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID   XPAR_FABRIC_XTMRCTR_0_INTR
#define INTC_DEVICE_ID        XPAR_INTC_0_DEVICE_ID
#define GPIO_INTERRUPT_ID     XPAR_FABRIC_XGPIO_0_INTR

/// Motor defines
#define MOTOR_FORWARD         0 // this is just a guess. if forward != 0, then its 1
#define MOTOR_REVERSE         1 // this is just a guess. if reverse != 1, then its 0
#define MOTOR_MAX_SPEED       100
#define MOTOR_MIN_SPEED       0

#define SONAR_THRESHOLD_VALUE 68000

////////////////////////////////////////////////////////////////////////////////////
/// FUNCTION PROTOTYPES

/// ISR and delay functions
//void timer_ISR(void *CallBackRef, u8 TmrCtrNumber);
void delay(int ms);

/// Helper functions
int      platform_init();
void     executionFailed();
void     setupTasks();
uint32_t getSonarDistance();

/// Tasks
void     taskSupervisor();
void     taskSonar();
void     taskIR();
void     taskMotor();
void     taskPivot();

////////////////////////////////////////////////////////////////////////////////////
/// Enums, structs and global variables

// Enumerated type for the tasks
typedef enum
{
  TASK_SUPERVISOR,
  TASK_SONAR,
  TASK_IR,
  TASK_MOTOR,
  TASK_PIVOT,
  MAX_TASKS
} task_t;

// Enumerated type for states -- i.e. (move forward, turn left, turn right, etc)
typedef enum
{
  STATE_IDLE,
  STATE_SETTING_SPEED,
  STATE_MEASURE_SONAR,
  STATE_MEASURE_IR,
  STATE_PIVOT_OBSTACLE,
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
TCB_t     *queue[MAX_TASKS];

// Timer counter -- multiply by TIMER_PERIOD_US to get time elapsed since system boot in microseconds
uint32_t  SystemCount;

// State variable and pivot flag
state     currentState;
_Bool     pivotRequired;

// Motor Controller, Global Speed Variables
PmodDHB1* MotorController;
u8        LeftMotorSpeed;   // This variable's value will be constantly applied to the left motor
u8        RightMotorSpeed;  // This variable's value will be constantly applied to the right motor

// Hardware instances
XIntc InterruptController;  // Instance of the Interrupt Controller
XTmrCtr Timer;              // Instance of the Timer
XGpio lightGpio;            // Instance of the AXI_GPIO_2
XGpio LEDGpio; 

////////////////////////////////////////////////////////////////////////////////////
/// ISRs and Delay functions

void timer_ISR(void *CallBackRef, u8 TmrCtrNumber)
{
  // Increment system time
  SystemCount++; // 1 unit = 0.5ms

  // Get instance of the timer linked to the interrupt
  XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

  // Check if the timer counter has expired
  if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber))
    // Queue our supervisor to run
    queue[TASK_SUPERVISOR]->taskReady = TRUE;
}

void delay(int ms){
  static int count;
  count = ms;

  // TODO: implement delay based on..?
}

////////////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS

/*---------------------------------------------------------------------------------------
    getSonarDistance()

        gets the raw sensor value from the two sonar sensors, averages it, then puts that
        average into a simple moving average for filtering.

---------------------------------------------------------------------------------------*/

// uint32_t getSonarDistance() {
//     static uint32_t movingAvgData[4] = {0};
//     static uint32_t staleIndex = 0;
//     static _Bool isFull = FALSE;

//     uint32_t dist = MAXSONAR_getDistance(&sonar, 1);
//     uint32_t dist2 = MAXSONAR_getDistance(&sonar, 2);

//     movingAvgData[staleIndex] = ((dist + dist2) / 2);
//     staleIndex = (staleIndex + 1) % 4;

//     if (staleIndex == 0) {
//         isFull = TRUE;
//     }

//     if (!isFull) {
//         return (SONAR_THRESHOLD_VALUE + 1000);
//     }
//     else {
//         uint32_t sma = 0;
//         for (int i = 0; i < 4; i++) {
//             sma += movingAvgData[i];
//         }
//         sma = sma / 4;
//         return sma;
//     }
//}

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

void testSonar(unsigned int* rgbLEDsData) {
  // Initialize sonar instances
  PMOD_DUAL_MAXSONAR sonar = {Dual_MAXSONAR_0_BASEADDR + MAXSONAR_CHANNEL_1_OFFSET, CLK_FREQ, 0};

  // Start sonar instances
  MAXSONAR_begin(&sonar, Dual_MAXSONAR_0_BASEADDR, CLK_FREQ);

  while (1)
  {
    // NOTE:
    // The lab8 doc on HThreads notes that getDistance returns the value in inches
    // However, the original lab8 example that uses getDistance implies its in cm
    // we need to test and see what it actually is.

    u64 distance0 = MAXSONAR_getDistance(&sonar, MAXSONAR_1);
    u64 distance1 = MAXSONAR_getDistance(&sonar, MAXSONAR_2);

    if(distance0 >= SONAR_THRESHOLD_VALUE){
      *rgbLEDsData = *rgbLEDsData | 0b0000000000100;  
    }
    if(distance1 >= SONAR_THRESHOLD_VALUE){
      *rgbLEDsData = *rgbLEDsData | 0b0000000100000;  
    }
    
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

  status = XGpio_SelfTest(&lightGpio);
  if(status != XST_SUCCESS){
      xil_printf("GPIO SelfTest Failed! Execution stopped.\n");
      executionFailed();
  }

  status = XGpio_Initialize(&LEDGpio, LS1_BASEADDR);
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize GPIO_2! Execution stopped.\n");
    executionFailed();
  }

  // Set GPIO_0 CHANNEL 2 as input
  XGpio_SetDataDirection(&LEDGpio, 0x1, 0x00);

  // Set GPIO_1 CHANNEL 1 as output
  XGpio_SetDataDirection(&LEDGpio, 0x2, 0xFF);

  status = XGpio_SelfTest(&lightGpio);
  if(status != XST_SUCCESS){
      xil_printf("GPIO SelfTest Failed! Execution stopped.\n");
      executionFailed();
  }

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
  queue[TASK_SUPERVISOR] =              malloc(sizeof(TCB_t));
  queue[TASK_SUPERVISOR]->taskPtr =     taskSupervisor;
  queue[TASK_SUPERVISOR]->taskDataPtr = NULL;
  queue[TASK_SUPERVISOR]->taskReady =   FALSE;

  // Task 1: taskSonar
  queue[TASK_SONAR] =                   malloc(sizeof(TCB_t));
  queue[TASK_SONAR]->taskPtr =          taskSonar;
  queue[TASK_SONAR]->taskDataPtr =      NULL;
  queue[TASK_SONAR]->taskReady =        TRUE;

  // Task 2: taskIR
  queue[TASK_IR] =                      malloc(sizeof(TCB_t));
  queue[TASK_IR]->taskPtr =             taskIR;
  queue[TASK_IR]->taskDataPtr =         NULL;
  queue[TASK_IR]->taskReady =           FALSE;

  // Task 3: taskMotor
  queue[TASK_MOTOR] =                   malloc(sizeof(TCB_t));
  queue[TASK_MOTOR]->taskPtr =          taskMotor;
  queue[TASK_MOTOR]->taskDataPtr =      NULL;
  queue[TASK_MOTOR]->taskReady =        FALSE;

  // Task 4: taskPivot
  queue[TASK_PIVOT] =                   malloc(sizeof(TCB_t));
  queue[TASK_PIVOT]->taskPtr =          taskPivot;
  queue[TASK_PIVOT]->taskDataPtr =      NULL;
  queue[TASK_PIVOT]->taskReady =        FALSE;
}

////////////////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTION

/// RGB LED data & tri
unsigned *rgbLEDsData =             RBG_LEDS_REG;
unsigned *rgbLEDsTri =              RBG_LEDS_REG + 1;
/// Infrared data & tri
volatile u32 *InfraredData =        (u32 *)LS1_BASEADDR + XGPIO_DATA_OFFSET;
volatile u32 *InfraredTristateReg = (u32 *)LS1_BASEADDR + XGPIO_TRI_OFFSET;

int main(int argc, char const *argv[])
{
  // Initialize timer counter, motor speeds, state, and tristate registers
  SystemCount =           0;
  pivotRequired =         FALSE;
  LeftMotorSpeed =        MOTOR_MIN_SPEED;
  RightMotorSpeed =       MOTOR_MIN_SPEED;
  currentState =          STATE_IDLE;
  *InfraredTristateReg =  0xF;
  *rgbLEDsTri =           0x0;

  // Setup the GPIO, Interrupt Controller and Timer
  int status = XST_FAILURE;
  status = platform_init();
  if (status != XST_SUCCESS)
  {
    xil_printf("Failed to initialize the platform! Execution stopped.\n");
    executionFailed();
  }

  // Initialize task queue and all of its tasks
  //setupTasks();

  testSonar(rgbLEDsData);

  // Main loop
  //while (1)
  //{
    // Iterate through task queue, execute 'ready' tasks
    // for (int i = 0; i < MAX_TASKS; i++)
    // {
    //   // Execute tasks that are only 'ready'
    //   if (queue[i]->taskReady)
    //   {
    //     // Execute the task
    //     (*(queue[i]->taskPtr))(queue[i]->taskDataPtr);

    //     // Reset the task ready flag
    //     queue[i]->taskReady = 0;
    //   }
    // }
  //}

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// TASK IMPLEMENTATIONS

/// Supervisor task
void taskSupervisor() {
  switch(currentState)
  {
    case STATE_IDLE:

      /// STATE_IDLE LOGIC ///
        // look for some activation signal (i.e. button press, idk man)
        // start round-robin FSM
      /// END OF LOGIC ///

      /// TEST CODE BELOW
      queue[TASK_SONAR]->taskReady = TRUE;
      break;

    case STATE_SETTING_SPEED:
      queue[TASK_MOTOR]->taskReady = TRUE;                  // Update motor speeds
      currentState =                 STATE_MEASURE_IR;      // next task is measuring IR
      break;

    case STATE_MEASURE_IR:
      queue[TASK_IR]->taskReady =    TRUE;                  // Measure IR (did we cross the tape)
      currentState =                 STATE_MEASURE_SONAR;   // next task is measuring SONAR

      // TODO: add logic w/ pivotRequired flag to determine pivot direction
      break;

    case STATE_MEASURE_SONAR:
      queue[TASK_SONAR]->taskReady = TRUE;                  // Measure SONAR (are we hitting an obstacle)
      currentState =                 STATE_SETTING_SPEED;   // next task is setting speed
      break;

    case STATE_PIVOT_OBSTACLE:
      queue[TASK_PIVOT]->taskReady = TRUE;                  // Pivot to avoid obstacle
      currentState =                 STATE_SETTING_SPEED;   // next task is setting speed after pivot
      break;

    default:
      // Invalid state go idle silly
      currentState = STATE_IDLE;
      break;
  }
}

// Take infrared measurements, adjust speed accordingly
void taskIR() {
  // If robot is veering to the right (left sensor touches reflective tape)
  if (*InfraredData & IR_L_SENSOR) {
      // Disable left motor, set right motor to 45% (turning left motion)
      LeftMotorSpeed = MOTOR_MIN_SPEED;
      RightMotorSpeed = 45;
  }

  // If robot is veering to the left (right sensor touches reflective tape)
  else if (*InfraredData & IR_R_SENSOR) {
      // Disable right motor, set left motor to 45% (turning right motion)
      RightMotorSpeed = MOTOR_MIN_SPEED;
      LeftMotorSpeed = 45;
  }
}

// Update motor speeds
void taskMotor() {
  // TODO: Check if DHB1/PWM is initialized

  // Set motor speed according to global vars
  DHB1_setMotorSpeeds(MotorController, LeftMotorSpeed, RightMotorSpeed);
}

// Take sonar measurements, stop robot if necessary
void taskSonar() {
  // Get sonar distance
  uint32_t distance = getSonarDistance();

  // If sonar distance is below the threshold, an obstacle is incoming (stop the robot pls)
  if (distance < SONAR_THRESHOLD_VALUE) {
      LeftMotorSpeed = MOTOR_MIN_SPEED;
      RightMotorSpeed = MOTOR_MIN_SPEED;

      // Update pivot flag to instruct robot to avoid obstacle
      pivotRequired = TRUE;
  }
}

// Pivot and avoid obstacle
void taskPivot() {

  // README: more logic will be added later on to determine the correct direction to pivot, so we stay on the line

  // Disable left motor, set right motor to 45% (turning left motion)
  LeftMotorSpeed = MOTOR_MIN_SPEED;
  RightMotorSpeed = 45;

  // TODO: Disable flag if pivot completed
  //pivotRequired = FALSE;
}