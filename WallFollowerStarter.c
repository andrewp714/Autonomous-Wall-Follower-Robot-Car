// WallFollowerStarter.c
// Runs on TM4C123
// Starter file for CECS 347 Project 2 - Wall Follower Robot
// Uses PWM to control two DC motors, SysTick timer for sampling rate,
// ADC to collect analog inputs from Sharp IR sensors and a potentiometer.
// Two IR distance sensors are mounted to measure
// distances to the left and right walls. An optional third IR sensor
// looks directly forward to avoid head-on collisions.
// The goal is to control wheel power to keep left and right wall distances equal.
// If an obstacle is too close to the front, both wheels stop immediately.
/*
    ------------------------------------------wall---------
                      /
                     /
                    / 
                   /
         -----------
         |         |
         | Robot   | ---> direction of motion and third sensor
         |         |
         -----------
                   \
                    \
                     \
                      \
    ------------------------------------------wall---------
*/
// The original project is provided by Dr. Daniel Valvano, Jonathan Valvano
// September 12, 2013
// Modification is made by Dr. Min He to provide this starter project.

// Hardware Connections:
// PE1: Forward-facing IR distance sensor
// PE4: Forward-right IR distance sensor
// PE5: Forward-left IR distance sensor
// PB5-2: L298N motor driver direction pins
// PB7-6: PWM signals for motor speed control
// PF3-1: Onboard LEDs for status indication
// PF4,0: Push buttons for user input

#include "ADCMultiSamples.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "stdint.h"

// External functions defined in startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // Low Power Mode

// Global variable to control sampling
uint8_t sample=0;

// ADC thresholds for IR sensor distances (calculated from datasheet and tested)
#define CRASH             IR15CM  // ADC output for 15cm: 1.8V -> (1.8/3.3)*4095=2233
#define IR15CM            2203  // ADC output for 15cm
#define IR20CM            1710  // ADC output for 20cm: 1.39V -> (1.39/3.3)*4095=1724
#define IR30CM            1086  // ADC output for 30cm: 0.9V -> (0.9/3.3)*4095=1116
#define IR40CM            847   // ADC output for 40cm: 0.74V -> (0.74/3.3)*4095=918
#define IR80CM            466   // ADC output for 80cm: 0.4V -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel

// Motor control constants
#define LEFTMIN           30    // Minimum percent duty cycle for left wheel (10-90)
#define LEFTMAX           50    // Maximum percent duty cycle for left wheel (10-90)
#define RIGHTCONST        40    // Constant percent duty cycle for right wheel (10-90)
#define TOTAL_PERIOD      80000 // PWM period in clock cycles
#define START_SPEED       80000 // Initial PWM duty cycle

// Motor direction control (L298N driver, PB5-2)
#define WHEEL_DIR (*((volatile unsigned long *)0x400050F0)) // PB5-2 direction pins
#define FORWARD           0x28  // Both motors forward
#define BACKWARD          0x14  // Both motors backward
#define LEFTPIVOT         0x18  // Pivot left
#define RIGHTPIVOT        0x24  // Pivot right

// LED control (PF3-1)
#define LED (*((volatile unsigned long *)0x40025038)) // PF3-1 for onboard LEDs
#define Dark              0x00  // All LEDs off
#define Red               0x02  // Red LED on
#define Blue              0x04  // Blue LED on
#define Green             0x08  // Green LED on
#define Yellow            0x0A  // Yellow LED (Red + Green)
#define Cran              0x0C  // Cyan LED (Blue + Green)
#define White             0x0E  // White LED (Red + Blue + Green)
#define Purple            0x06  // Purple LED (Red + Blue)
#define NVIC_EN0_PORTF    0x40000000 // NVIC enable for Port F


// Function prototypes
void System_Init(void);                    // Initialize system components
void Delay(void);                          // Delay for 0.25 seconds
void LED_Init(void);                       // Initialize PF3-1 for LEDs
void Buttons_Init(void);                   // Initialize PF4,0 for push buttons
void Motor_Init(void);                     // Initialize motor control pins
void SysTick_Init(void);                   // Initialize SysTick timer
void PWM_PB76_Init(void);                  // Initialize PWM for PB7-6
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R); // Set PWM duty cycles
void steering(uint16_t ahead_dist, uint16_t right_dist, uint16_t left_dist); // Control steering
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8); // FIR filter for ADC
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8); // IIR filter for ADC
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3); // Calculate median of three values
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8); // Median filter for ADC

// Main program
int main(void) {
  uint16_t left, right, ahead; // ADC values for left, right, and front sensors

  // Initialize system
  DisableInterrupts();  // Disable interrupts during initialization
  System_Init();        // Configure PLL, SysTick, ADC, LEDs, buttons, PWM, and motors
  EnableInterrupts();   // Enable interrupts after initialization

  // Set initial motor speed and direction
  PWM_PB76_Duty(9000, 9000); // Set initial duty cycle for both wheels
  LED = Yellow;              // Indicate moving forward
  WHEEL_DIR = FORWARD;       // Set motors to move forward
  PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
  Delay();                   // Wait for 0.25 seconds

  // Main control loop
  while (1) {
    if (sample) { // Check if new sample is ready
      sample = 0; // Reset sample flag
      ReadADCMedianFilter(&ahead, &right, &left); // Read and filter sensor data
      steering(ahead, right, left); // Adjust motor control based on sensor data
    }
  }
}

// Initialize all system components
void System_Init(void) {
  PLL_Init();           // Set system clock to 80 MHz
  SysTick_Init();       // Configure SysTick for periodic interrupts
  ADC_Init298();        // Initialize ADC for AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LED_Init();           // Configure onboard LEDs
  Buttons_Init();       // Configure onboard push buttons
  PWM_PB76_Init();      // Initialize PWM for motor speed control
  Motor_Init();         // Initialize motor direction control pins
}

// Initialize push buttons on PF4, PF0 with falling edge interrupts
void Buttons_Init(void) {
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // Unlock Port F
  GPIO_PORTF_CR_R |= 0x11;          // Allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;        // Set PF4,0 as inputs
  GPIO_PORTF_AFSEL_R &= ~0x11;      // Disable alternate functions
  GPIO_PORTF_DEN_R |= 0x11;         // Enable digital I/O
  GPIO_PORTF_PCTL_R &= ~0x000F000F; // Configure as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;      // Disable analog functionality
  GPIO_PORTF_PUR_R |= 0x11;         // Enable weak pull-up resistors
  GPIO_PORTF_IS_R &= ~0x11;         // Edge-sensitive interrupts
  GPIO_PORTF_IBE_R &= ~0x11;        // Single-edge interrupts
  GPIO_PORTF_IEV_R &= ~0x11;        // Falling edge interrupts
  GPIO_PORTF_ICR_R = 0x11;          // Clear interrupt flags
  GPIO_PORTF_IM_R |= 0x11;          // Enable interrupts on PF4,0
  NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00400000; // Set Port F interrupt priority to 2
  NVIC_EN0_R |= NVIC_EN0_PORTF;     // Enable Port F interrupt
}


// Port F Initialization PF3-1 (Initialize onboard LEDs)
void LED_Init(void) {
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // Activate Port F clock
  while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOF) == 0) {} // Wait for clock
  GPIO_PORTF_AMSEL_R &= ~0x0E;      // Disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // Configure as GPIO
  GPIO_PORTF_DIR_R |= 0x0E;         // Set PF3-1 as outputs
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // Disable alternate functions
  GPIO_PORTF_DEN_R |= 0x0E;         // Enable digital I/O
  LED = Dark;                       // Turn off all LEDs
}

// Initialize motor direction control pins on PB5-2
void Motor_Init(void) {
  if ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB) == 0) {
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // Activate Port B clock
    while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB) == 0) {} // Wait for clock
  }
  GPIO_PORTB_AMSEL_R &= ~0x3C;      // Disable analog function
  GPIO_PORTB_AFSEL_R &= ~0x3C;      // Disable alternate functions
  GPIO_PORTB_PCTL_R &= ~0x00FFFF00; // Configure as GPIO
  GPIO_PORTB_DIR_R |= 0x3C;         // Set PB5-2 as outputs
  GPIO_PORTB_DEN_R |= 0x3C;         // Enable digital I/O
}

// Initialize SysTick timer for periodic sampling
void SysTick_Init(void) {
  NVIC_ST_CTRL_R = 0;                   // Disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // Set maximum reload value
  NVIC_ST_CURRENT_R = 0;                // Clear current value
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x20000000; // Set priority
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC + NVIC_ST_CTRL_INTEN; // Enable SysTick
}

// Initialize PWM module 0, generator 0, outputs A&B (PB7-6) for
// L298N motor driver allowing for a variable speed of robot car.
void PWM_PB76_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB)==0){}; // Wait for clock
	}
	GPIO_PORTB_AFSEL_R |= 0xC0;         // Enable alternate function for PB7-6 for PWM
  GPIO_PORTB_PCTL_R &= ~0xFF000000;   // PWM to be used (Clear PCTL for PWM)
  GPIO_PORTB_PCTL_R |= 0x44000000;    // PWM to be used (Configure PB7-6 for PWM)
  GPIO_PORTB_DEN_R |= 0xC0;           // Enable digital I/O 
	// Initializes PWM settings
	SYSCTL_RCGCPWM_R |= 0x01;          // Activate PWM0 clock
	SYSCTL_RCC_R &= ~0x001E0000;       // Clear any previous PWM divider values
	// PWM0_0 output A&B Initialization for PB76
  PWM0_0_CTL_R = 0;                // Disable PWM during setup
  PWM0_0_GENA_R |= 0xC8;           // Set PWM output A behavior
  PWM0_0_GENB_R |= 0xC08;          // Set PWM output B behavior
  PWM0_0_LOAD_R = TOTAL_PERIOD - 1; // Set PWM period
  PWM0_0_CMPA_R = 0;               // Initialize compare A
  PWM0_0_CMPB_R = 0;               // Initialize compare B
	
	PWM0_0_CTL_R |= 0x00000001;	      // Enable PWM0 Generator 0 in Countdown mode
	PWM0_ENABLE_R &= ~0x00000003;	    // Disable PB76: PWM0 output 0&1 on initialization
}

// Dependency: PWM_Init()
// Inputs: 
//	duty_L is the value corresponding to the duty cycle of the left wheel
//	duty_R is the value corresponding to the duty cycle of the right wheel
// Outputs: None 
// Description: Changes the duty cycles of PB76 by changing the CMP registers
// Set PWM duty cycles for left (PB6) and right (PB7) motors
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R) {
  PWM0_0_CMPA_R = duty_L - 1; // Set left motor duty cycle
  PWM0_0_CMPB_R = duty_R - 1; // Set right motor duty cycle
}


// Control robot steering based on sensor distances
void steering(uint16_t ahead_dist, uint16_t right_dist, uint16_t left_dist) {
  // Steering logic:
  // 1. Stop if any sensor detects obstacle < 20cm
  // 2. Stop if all sensors detect obstacles > 40cm
  // 3. Turn right if left sensor detects obstacle within 30cm
  // 4. Turn left if right sensor detects obstacle within 30cm
  // 5. Go straight if no obstacles within 30cm

  WHEEL_DIR = FORWARD; // Default direction
  if (left_dist > CRASH) { // Obstacle too close on left
    LED = Red;             // Indicate critical distance
    WHEEL_DIR = RIGHTPIVOT; // Pivot right
    PWM_PB76_Duty(9000, 9000); // Set duty cycles
    PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
  } else if (left_dist > IR30CM) { // Obstacle on left within 30cm
    LED = Green;             // Indicate right turn
    WHEEL_DIR = RIGHTPIVOT;  // Pivot right
    PWM_PB76_Duty(10500, 6500); // Adjust duty cycles
    PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
  } else if (right_dist > CRASH) { // Obstacle too close on right
    LED = Red;             // Indicate critical distance
    WHEEL_DIR = LEFTPIVOT; // Pivot left
    PWM_PB76_Duty(9000, 9000); // Set duty cycles
    PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
  } else if (right_dist > IR40CM) { // Obstacle on right within 40cm
    LED = Blue;            // Indicate left turn
    WHEEL_DIR = LEFTPIVOT; // Pivot left
    PWM_PB76_Duty(6500, 11000); // Adjust duty cycles
    PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
  } else { // No obstacles detected
    LED = Dark;            // Turn off LEDs
    PWM_PB76_Duty(8500, 8500); // Set equal duty cycles
    WHEEL_DIR = FORWARD;   // Move forward
    PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
  }
}

// SysTick interrupt handler to trigger sampling
void SysTick_Handler(void) {
  sample = 1; // Set flag to indicate new sample
}

// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an FIR filter:
// y(n) = (x(n) + x(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8) {
  static uint16_t ain2previous = 0; // Previous AIN2 value
  static uint16_t ain9previous = 0; // Previous AIN9 value
  static uint16_t ain8previous = 0; // Previous AIN8 value
  uint16_t ain2newest, ain9newest, ain8newest; // New ADC samples
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // Sample ADC channels
  *ain2 = (ain2newest + ain2previous) / 2; // Average new and previous
  *ain9 = (ain9newest + ain9previous) / 2;
  *ain8 = (ain8newest + ain8previous) / 2;
  ain2previous = ain2newest; // Update previous values
  ain9previous = ain9newest;
  ain8previous = ain8newest;
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an IIR filter:
// y(n) = (x(n) + y(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8) {
  static uint16_t filter2previous = 0; // Previous filtered AIN2
  static uint16_t filter9previous = 0; // Previous filtered AIN9
  static uint16_t filter8previous = 0; // Previous filtered AIN8
  uint16_t ain2newest, ain9newest, ain8newest; // New ADC samples
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // Sample ADC channels
  *ain2 = filter2previous = (ain2newest + filter2previous) / 2; // Update filtered values
  *ain9 = filter9previous = (ain9newest + filter9previous) / 2;
  *ain8 = filter8previous = (ain8newest + filter8previous) / 2;
}

// Median function from EE345M Lab 7 2011; Program 5.1 from Volume 3
// helper function for ReadADCMedianFilter() but works for general use
// Calculate median of three values for filtering
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3) {
  uint16_t result;
  if (u1 > u2) {
    if (u2 > u3) result = u2;      // u1>u2>u3
    else if (u1 > u3) result = u3;  // u1>u3>u2
    else result = u1;               // u3>u1>u2
  } else {
    if (u3 > u2) result = u2;       // u3>u2>u1
    else if (u1 > u3) result = u1;  // u2>u1>u3
    else result = u3;               // u2>u3>u1
  }
  return result;
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is a median filter:
// y(n) = median(x(n), x(n-1), x(n-2))
// Assumes: ADC initialized by previously calling ADC_Init298()
// Median filter for ADC readings: y(n) = median(x(n), x(n-1), x(n-2))
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8) {
  static uint16_t ain2oldest = 0, ain2middle = 0; // AIN2 history
  static uint16_t ain9oldest = 0, ain9middle = 0; // AIN9 history
  static uint16_t ain8oldest = 0, ain8middle = 0; // AIN8 history
  uint16_t ain2newest, ain9newest, ain8newest; // New ADC samples
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // Sample ADC channels
  *ain2 = median(ain2newest, ain2middle, ain2oldest); // Calculate medians
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle; // Update history
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}

// Delay for 0.25 seconds
void Delay(void) {
  unsigned long volatile time = 727240 * 500 / 91; // Calibrated for 0.25s
  while (time) {
    time--;
  }
}
