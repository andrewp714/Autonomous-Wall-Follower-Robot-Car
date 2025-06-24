// WallFollowerStarter.c
// Runs on TM4C123
// This is the starter file for CECS 347 Project 2 - A Wall Follower
// This project use PWM to control two DC Motors, SysTick timer 
// to control sampling rate, ADC to collect analog inputs from
// Sharp IR sensors and a potentiometer.
// Two GP2Y0A21YK0F analog IR distance sensors are used to allow
// the robot to follow a wall. A Minimum of two IR sensors are mounted
// looking forward to the left and forward to the right. 
// A optional third IR sensor looks directly forward can be used to avoid
// a head-on collision. Basically, the goal is to control power to each wheel so the
// left and right distances to the walls are equal.
// If an object is detected too close to the front of the robot,
// both wheels are immediately stopped.
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

// PE1 connected to forward facing IR distance sensor
// PE4 connected to forward right IR distance sensor
// PE5 connected to forward left IR distance sensor

#include "ADCMultiSamples.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "stdint.h"

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

uint8_t sample=0;

// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR15CM            2203  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1710  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1086  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            847   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            466   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTMIN        30    // minimum percent duty cycle of left wheel (10 to 90)
#define LEFTMAX        50    // maximum percent duty cycle of left wheel (10 to 90)
#define RIGHTCONST     40    // constant percent duty cycle of right wheel (10 to 90)

//  Period 
#define TOTAL_PERIOD 80000 
#define START_SPEED 80000

//  Car
#define WHEEL_DIR (*((volatile unsigned long *)0x400050F0)) // PB5432 are the four direction pins for L298
#define FORWARD 0x28
#define BACKWARD 0x14
#define LEFTPIVOT 0x18
#define RIGHTPIVOT 0x24

//  LEDS
#define LED (*((volatile unsigned long *)0x40025038))  // use onboard three LEDs: PF321
#define Dark    	0x00
#define Red     	0x02
#define Blue    	0x04
#define Green   	0x08
#define Yellow  	0x0A
#define Cran      0x0C
#define White   	0x0E
#define Purple  	0x06
#define NVIC_EN0_PORTF 0x40000000


void System_Init(void);
void Delay(void);
void LED_Init(void);		 //  PF3-1
void Buttons_Init(void); //  PF4,0
void Motor_Init(void);
void SysTick_Init(void);
void PWM_PB76_Init(void);
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3);
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and


int main(void){
  uint16_t left, right, ahead;
  
  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
  EnableInterrupts();   // enable after all initialization are done

	 

  PWM_PB76_Duty(9000, 9000);
    
	// moving forward
	LED = Yellow;
	WHEEL_DIR = FORWARD;
	
	// Loop for 2 seconds

	
	PWM0_ENABLE_R |= 0x00000003; // enable both wheels
	Delay();
	
  while(1){
    if(sample) {
      sample = 0;
      // choose one of the software filter methods
      ReadADCMedianFilter(&ahead, &right, &left);
      steering(ahead,right,left);
    }
  }
}

void System_Init(void) {
  PLL_Init();           // bus clock at 80 MHz
  SysTick_Init();       // Initialize SysTick timer with interrupt for smapling control
  ADC_Init298();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LED_Init();         	// configure onboard LEDs and push buttons
	Buttons_Init();				//  Configure Onboard Push Buttons
	PWM_PB76_Init();
  Motor_Init();         // Initialize signals for the two DC Motors
}

// Initilize port F and arm PF4, PF0 for falling edge interrupts
void Buttons_Init(void)
{  
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; 	// unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x11;         		// allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    			// (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  			//     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;     			//     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; 	//  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  			//     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     			//     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     			// (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    			//     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    			//     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      			// (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      			// (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R |= NVIC_EN0_PORTF;      // (h) enable interrupt 30 in NVIC
}


// Port F Initialization PF3-1
void LED_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;	// Activate F clocks
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)==0){};
		
  GPIO_PORTF_AMSEL_R &= ~0x0E;      // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;         // 6) PF1-PF3 output
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 7) no alternate function     
  GPIO_PORTF_DEN_R |= 0x0E;         // 8) enable digital pins PF3-PF1
  LED = Dark;                       // Turn off all LEDs.
}

//  PB5-2
void Motor_Init(void){
	
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
			SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
			while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
		}
			
		GPIO_PORTB_AMSEL_R &= ~0x3C;	// disable analog function
		GPIO_PORTB_AFSEL_R &= ~0x3C;	// no alternate function
		GPIO_PORTB_PCTL_R &= ~0x00FFFF00;	// GPIO clear bit PCTL 
		GPIO_PORTB_DIR_R |= 0x3C; // output on pin(s)
		GPIO_PORTB_DEN_R |= 0x3C;	// enable digital I/O on pin(s)
}

void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value 
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x20000000;
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC+NVIC_ST_CTRL_INTEN;
}

// Initializes the PWM module 0 generator 0 outputs A&B tied to PB76 to be used with the 
//		L298N motor driver allowing for a variable speed of robot car.
void PWM_PB76_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
	
	GPIO_PORTB_AFSEL_R |= 0xC0;	// enable alt funct: PB76 for PWM
  GPIO_PORTB_PCTL_R &= ~0xFF000000; // PWM to be used
  GPIO_PORTB_PCTL_R |= 0x44000000; // PWM to be used
  GPIO_PORTB_DEN_R |= 0xC0;	// enable digital I/O 
	
	// Initializes PWM settings
	SYSCTL_RCGCPWM_R |= 0x01;	// activate PWM0
	SYSCTL_RCC_R &= ~0x001E0000; // Clear any previous PWM divider values
	
	// PWM0_0 output A&B Initialization for PB76
	PWM0_0_CTL_R = 0;	// re-loading down-counting mode
	PWM0_0_GENA_R |= 0xC8;	// low on LOAD, high on CMPA down
	PWM0_0_GENB_R |= 0xC08;// low on LOAD, high on CMPB down
	PWM0_0_LOAD_R = TOTAL_PERIOD - 1;	// cycles needed to count down to 0
  PWM0_0_CMPA_R = 0;	// count value when output rises
	PWM0_0_CMPB_R = 0;	// count value when output rises
	
	PWM0_0_CTL_R |= 0x00000001;	// Enable PWM0 Generator 0 in Countdown mode
	PWM0_ENABLE_R &= ~0x00000003;	// Disable PB76:PWM0 output 0&1 on initialization
}

// Dependency: PWM_Init()
// Inputs: 
//	duty_L is the value corresponding to the duty cycle of the left wheel
//	duty_R is the value corresponding to the duty cycle of the right wheel
// Outputs: None 
// Description: Changes the duty cycles of PB76 by changing the CMP registers
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R){
	PWM0_0_CMPA_R = duty_L - 1;	// PB6 count value when output rises
  PWM0_0_CMPB_R = duty_R - 1;	// PB7 count value when output rises
}


void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist){
  // Suggest the following simple control as starting point:
  // 1. If any one of the senors see obstacle <20cm, stop
  // 2. If all sensors detect no obstacle within 35cm, stop
  // 3. If left sees obstacle within 30cm, turn right
  // 4. If right sees obstacle within 30cm, turn left
  // 5. If both sensors see no obstacle within 30cm, go straight  
  // Feel free to add more controlls to fine tune your robot car.
  // Make sure to take care of both wheel movements and LED display here.
	
	//  1 Stop if less than 20
	
//	if (ahead_dist < IR20CM || left_dist < IR20CM || right_dist < IR20CM)
//	{
//		PWM0_ENABLE_R &= ~0x00000003;  //  Disable Both Wheels
//	}
	
	// 2 Stop if more than 40
//	else if (ahead_dist < IR40CM && left_dist <  IR40CM && right_dist < IR40CM)
//	{
//		PWM0_ENABLE_R &= ~0x00000003;  //  Disable Both Wheels
//	}
//  
	
	WHEEL_DIR = FORWARD;
	//  3 Turn Right
	if (left_dist > CRASH)
	{
		LED = Red;
		WHEEL_DIR = 0x24;
		PWM_PB76_Duty(9000,9000);
		PWM0_ENABLE_R |= 0x00000002; // Enable right wheel
		PWM0_ENABLE_R |= 0x00000001; // Enable left wheel
	}
	
	else if (left_dist > IR30CM)
	{
		LED = Green;
		WHEEL_DIR = RIGHTPIVOT;
		PWM_PB76_Duty(10500,6500);
		PWM0_ENABLE_R |= 0x00000002; // Enable right wheel
		PWM0_ENABLE_R |= 0x00000001; // Enable left wheel
	}
	
	else if (right_dist > CRASH)
	{
		LED = Red;
		WHEEL_DIR = 0x18;
		PWM_PB76_Duty(9000,9000);
		PWM0_ENABLE_R |= 0x00000002; // Enable right wheel
		PWM0_ENABLE_R |= 0x00000001; // Enable left wheel
	}
	// 4 Turn Left
	else if (right_dist > IR40CM)
	{

		LED = Blue;
		WHEEL_DIR = LEFTPIVOT;
		PWM_PB76_Duty(6500,11000);
		PWM0_ENABLE_R |= 0x00000001; // Enable left wheel
		PWM0_ENABLE_R |= 0x00000002; // Enable right wheel
	}
	
	//  5 Go straight if no obstacle 
	else 
	{
		LED = Dark;
		PWM_PB76_Duty(8500,8500);
		WHEEL_DIR = FORWARD;
		PWM0_ENABLE_R |= 0x00000003; // enable both wheels
		
	}
	
}

void SysTick_Handler(void){
  sample = 1;
}

// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an FIR filter:
// y(n) = (x(n) + x(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  static uint16_t ain2previous=0; // after the first call, the value changed to 12
  static uint16_t ain9previous=0;
  static uint16_t ain8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = (ain2newest + ain2previous)/2;
  *ain9 = (ain9newest + ain9previous)/2;
  *ain8 = (ain8newest + ain8previous)/2;
  ain2previous = ain2newest; ain9previous = ain9newest; ain8previous = ain8newest;
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an IIR filter:
// y(n) = (x(n) + y(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   y(n-1)
  static uint16_t filter2previous=0;
  static uint16_t filter9previous=0;
  static uint16_t filter8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = filter2previous = (ain2newest + filter2previous)/2;
  *ain9 = filter9previous = (ain9newest + filter9previous)/2;
  *ain8 = filter8previous = (ain8newest + filter8previous)/2;
}

// Median function from EE345M Lab 7 2011; Program 5.1 from Volume 3
// helper function for ReadADCMedianFilter() but works for general use
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3){
uint16_t result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is a median filter:
// y(n) = median(x(n), x(n-1), x(n-2))
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   x(n-2)        x(n-1)
  static uint16_t ain2oldest=0, ain2middle=0;
  static uint16_t ain9oldest=0, ain9middle=0;
  static uint16_t ain8oldest=0, ain8middle=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}

// Subroutine to wait 0.25 sec

void Delay(void){
	unsigned long volatile time;
  time = 727240*500/91;  // 0.25sec
  while(time){
		time--;
  }
}