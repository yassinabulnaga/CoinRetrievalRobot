#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "adc.h"
#include "UART2.h"

#define F_CPU 32000000L
#define DEF_F 100000L // 10us tick

// Some 'defines' to turn pins on/off easily (pins must be configured as outputs)
#define PB3_0 (GPIOB->ODR &= ~BIT3) // Left Wheel Black
#define PB3_1 (GPIOB->ODR |=  BIT3)
// Left Wheel Red
#define PB4_0 (GPIOB->ODR &= ~BIT4) // Off
#define PB4_1 (GPIOB->ODR |=  BIT4) // On
// Right Wheel Red
#define PB5_0 (GPIOB->ODR &= ~BIT5) // Off
#define PB5_1 (GPIOB->ODR |=  BIT5) // On
// Right Wheel Black
#define PB6_0 (GPIOB->ODR &= ~BIT6) // Off
#define PB6_1 (GPIOB->ODR |=  BIT6) // On
// Electromagnet
#define PB7_0 (GPIOB->ODR &= ~BIT7) // Off
#define PB7_1 (GPIOB->ODR |=  BIT7) // On

#define PA13_0 (GPIOA->ODR &= ~BIT13) // Radio
#define PA13_1 (GPIOA->ODR |=  BIT13)

#define PA6_0 (GPIOA->ODR &= ~BIT6) // LED Debugging
#define PA6_1 (GPIOA->ODR |=  BIT6)
#define PA7_0 (GPIOA->ODR &= ~BIT7)
#define PA7_1 (GPIOA->ODR |=  BIT7)

// // A define to easily read PA14 (PA14 must be configured as input first)
// #define PA14 (GPIOA->IDR & BIT14)

// A define to easily read PA1 (PA1 must be configured as input first)
#define PA1 (GPIOA->IDR & BIT1)

// A define to easily read PA8 (PA8 must be configured as input first)
#define PA8 (GPIOA->IDR & BIT8)

// LQFP32 pinout
//                 ----------
//           VDD -|1       32|- VSS
//          PC14 -|2       31|- BOOT0
//          PC15 -|3       30|- PB7 (OUT 5) Electromagnet
//          NRST -|4       29|- PB6 (OUT 4) Right Wheel Black (Back)
//          VDDA -|5       28|- PB5 (OUT 3) Right Wheel Red (Forward)
//           PA0 -|6       27|- PB4 (OUT 2) Left Wheel Red (Back)
//  (button) PA1 -|7       26|- PB3 (OUT 1) Left Wheel Black (Forward)
//           PA2 -|8       25|- PA15 (Used for RXD of UART2, connects to TXD of JDY40)
//           PA3 -|9       24|- PA14 (Used for TXD of UART2, connects to RXD of JDY40)
//           PA4 -|10      23|- PA13 (Used for SET of JDY40)
//           PA5 -|11      22|- PA12 (pwm2)
//     (LED) PA6 -|12      21|- PA11 (pwm1)
//     (LED) PA7 -|13      20|- PA10 (Reserved for RXD)
// (ADC_IN8) PB0 -|14      19|- PA9  (Reserved for TXD)
// (ADC_IN9) PB1 -|15      18|- PA8  (Measure the period at this pin)
//           VSS -|16      17|- VDD
//                 ----------


/****************************************************************************************************************
BUTTON VALUE DEFINITIONS 
****************************************************************************************************************/

// Joystick Ranges
#define LEVEL0 0
#define LEVEL1 28
#define LEVEL2 55
#define LEVEL3 82
#define LEVEL4 110
#define LEVEL5 146
#define LEVEL6 174
#define LEVEL7 201
#define LEVEL8 228
#define LEVEL9 255

// Button Values 
#define BUTTON_NONE 255 
#define BUTTON_ERROR -1

// Buttons 2
#define SQUARE 127 
#define O_BUTTON 223 
#define TRIANGLE 239 
#define X_BUTTON 191 
#define R1 247 
#define R2 253 
#define L1 251 
#define L2 254 

// Buttons 1
#define START 247 
#define SELECT 254 
#define START_SELECT 250 
#define L3 253 
#define R3 251  
#define DPAD_UP 239 
#define DPAD_DOWN 191  
#define DPAD_LEFT 127  
#define DPAD_RIGHT 223  

// Additional definitions for manual control:
#define MAX_SPEED       100   // Maximum speed value.
#define MAX_TURN        50    // Maximum turning value.
#define SPEED_THRESHOLD 10    // Minimum speed to command a movement.

// ----- Define servo movement limits and step values -----
#define MIN_ISR_PWM1 60    // Vertical servo upper limit
#define MAX_ISR_PWM1 255   // Vertical servo lower limit
#define MIN_ISR_PWM2 60    // Horizontal servo left limit
#define MAX_ISR_PWM2 255   // Horizontal servo right limit
// Use larger steps and shorter delays for faster movement
#define VERTICAL_STEP 20        
#define HORIZONTAL_STEP 20      
#define VERTICAL_DELAY_MS 5   
#define HORIZONTAL_DELAY_MS 5 

/****************************************************************************************************************
GLOBAL VARIABLE DECLARATIONS
****************************************************************************************************************/

volatile int PWM_Counter = 0;
volatile unsigned char ISR_pwm1 = 60, ISR_pwm2 = 255;

// Could measure references at startup or hard-code values here
volatile long int reference_frequency;
volatile long int frequency;
volatile long int reference_count;
volatile long int count;
volatile int reference_voltage[2];
volatile int voltage[2];

//RemoteCommand * Controller_Data; //used to store data from PS2 controller
int * mode_flag; //used to check if robot should be in automatic pickup mode

// Global coin counter added for the game:
volatile int coin_count = 0;  //

// Structure to hold decoded remote command values
typedef struct {
    int b1;   // buttons1
    int b2;   // buttons2
    int rx;   // Right Joystick X
    int ry;   // Right Joystick Y
    int lx;   // Left Joystick X
    int ly;   // Left Joystick Y
} RemoteCommand;

/****************************************************************************************************************
FUNCTION PROTOTYPES
****************************************************************************************************************/

//Remote Decoding Functions
int decodeButton1(char ch);
int decodeButton2(char ch);
int decodeJoystick(char letter, int isYAxis);
int parseRemoteCommand(char *str, RemoteCommand *cmd);
int mapValue(int x, int in_min, int in_max, int out_min, int out_max);

//Robot Base Functions
void wait_1ms(void);
void waitms(int len);
void TIM2_Handler(void);
void Hardware_Init(void);
long int GetPeriod(int n);
void PrintNumber(long int val, int Base, int digits);

//Radio Functions
void SendATCommand(char *s);
void ReceptionOff(void);
void processRadioData(char *buff, RemoteCommand *currentCmd);
//int parseRemoteCommand(char *str, RemoteCommand *cmd);
int automode_toggle_check (char button);

//Movement Functions
void leftWheelForward(void);
void leftWheelBackward(void);
void leftWheelStop(void);
void rightWheelForward(void);
void rightWheelBackward(void);
void rightWheelStop(void);
void turnAround(void);
void turn90degreesCW(void);
void turn90DegreesCCW(void);
void aLittleForward(void);
void aLittleBackward(void);

//Mode Functions
void autodrive(RemoteCommand *currentCommand);
void gameMode(void);
void ManualControl(RemoteCommand *command);

//Electromagnetic Functions
int detectMetal(void);
int detectPerimeter(void);
void collectCoin(void);
void resetArm(void);

/****************************************************************************************************************
REMOTE COMMAND DECODING FUNCTIONS
Based on the Controller Data Encoding Table :contentReference[oaicite:2]{index=2}â€‹:contentReference[oaicite:3]{index=3}:
Data format: [b1Char][b2Char][rxChar][ryChar][lxChar][lyChar]
For example: "NN1515" means no buttons pressed, Right Joystick left & centre, Left Joystick left & centre.
****************************************************************************************************************/


int decodeButton1(char ch) {
    switch(ch) {
       case 'N': return BUTTON_NONE;   // No button pressed
       case 's': return SELECT;        // SELECT
       case 'S': return START;         // START
       case 'Z': return START_SELECT;  // START_SELECT       
       case 'U': return DPAD_UP;       // DPAD UP
       case 'D': return DPAD_DOWN;     // DPAD DOWN
       case 'L': return DPAD_LEFT;     // DPAD LEFT
       case 'R': return DPAD_RIGHT;    // DPAD RIGHT
       case 'l': return L3;            // L3 (joystick press)
       case 'r': return R3;            // R3 (joystick press)
       case 'E': return BUTTON_ERROR;  // Error
       default:  return 0;             // Unknown value
    }
}

int decodeButton2(char ch) {
    switch(ch) {
       case 'N': return BUTTON_NONE;   // No button pressed
       case 'X': return X_BUTTON;      // X
       case 'S': return SQUARE;        // Square
       case 'O': return O_BUTTON;      // O
       case 'T': return TRIANGLE;      // Triangle
       case 'r': return R1;            // R1
       case 'R': return R2;            // R2
       case 'l': return L1;            // L1
       case 'L': return L2;            // L2
       case 'E': return BUTTON_ERROR;  // Error
       default:  return 0;
    }
}

int decodeJoystick(char letter, int isYAxis) {
    if(letter < '1' || letter > '9') return -1; // Invalid letter
    int num = letter - '0';
    // For non-inverted joystick, compute midpoint using LEVEL definitions.
    static int mappingX[9] = {
        (LEVEL0 + LEVEL1) / 2,  // '1': (0+28)/2 = 14
        (LEVEL1 + LEVEL2) / 2,  // '2': (28+55)/2 = 41
        (LEVEL2 + LEVEL3) / 2,  // '3': (55+82)/2 = 68
        (LEVEL3 + LEVEL4) / 2,  // '4': (82+110)/2 = 96
        (LEVEL4 + LEVEL5) / 2,  // '5': (110+146)/2 = 128
        (LEVEL5 + LEVEL6) / 2,  // '6': (146+174)/2 = 160
        (LEVEL6 + LEVEL7) / 2,  // '7': (174+201)/2 = 187
        (LEVEL7 + LEVEL8) / 2,  // '8': (201+228)/2 = 214
        (LEVEL8 + LEVEL9) / 2   // '9': (228+255)/2 = 241 (integer division)
    };
    // For the y-axis, use an inverted mapping (reverse order).
    static int mappingY[9] = {
        (LEVEL8 + LEVEL9) / 2,  // '1' â†’ highest: 241
        (LEVEL7 + LEVEL8) / 2,  // '2': 214
        (LEVEL6 + LEVEL7) / 2,  // '3': 187
        (LEVEL5 + LEVEL6) / 2,  // '4': 160
        (LEVEL4 + LEVEL5) / 2,  // '5': 128
        (LEVEL3 + LEVEL4) / 2,  // '6': 96
        (LEVEL2 + LEVEL3) / 2,  // '7': 68
        (LEVEL1 + LEVEL2) / 2,  // '8': 41
        (LEVEL0 + LEVEL1) / 2   // '9' â†’ lowest: 14
    };
    if(!isYAxis) {
       return mappingX[num - 1];
    } else {
       return mappingY[num - 1];
    }
}

int parseRemoteCommand(char *str, RemoteCommand *cmd) {
    if(strlen(str) != 6) return 0;  // Expect exactly 6 characters
    cmd->b1 = decodeButton1(str[0]);
    cmd->b2 = decodeButton2(str[1]);
    cmd->rx = decodeJoystick(str[2], 0); // Right Joystick X
    cmd->ry = decodeJoystick(str[3], 1); // Right Joystick Y
    cmd->lx = decodeJoystick(str[4], 0); // Left Joystick X
    cmd->ly = decodeJoystick(str[5], 1); // Left Joystick Y
    return 1;
}

// Mapping function: maps an input value x from [in_min, in_max] to [out_min, out_max].
int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/****************************************************************************************************************
ROBOT BASE FUNCTIONS
****************************************************************************************************************/

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

// Interrupt service routines are the same as normal
// subroutines (or C funtions) in Cortex-M microcontrollers.
// The following should happen at a rate of 1kHz.
// The following function is associated with the TIM2 interrupt 
// via the interrupt vector table defined in startup.c
void TIM2_Handler(void) 
{
	TIM2->SR &= ~BIT0; // clear update interrupt flag
	PWM_Counter++;
	
	if(ISR_pwm1>PWM_Counter)
	{
		GPIOA->ODR |= BIT11;
	}
	else
	{
		GPIOA->ODR &= ~BIT11;
	}
	
	if(ISR_pwm2>PWM_Counter)
	{
		GPIOA->ODR |= BIT12;
	}
	else
	{
		GPIOA->ODR &= ~BIT12;
	}
	
	if (PWM_Counter > 2000) // THe period is 20ms
	{
		PWM_Counter=0;
		GPIOA->ODR |= (BIT11|BIT12);
	}
	
}


void Hardware_Init(void)
{
	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451
	//GPIOA->OSPEEDR |= 0xfc000000; // Pins PA15, PA14, PA13 configured for very high speed! Page 201 of RM0451

	RCC->IOPENR  |= (BIT1|BIT0);  // peripheral clock enable for ports A and B

	// Configure the pin used for analog input: PB0 and PB1 (pins 14 and 15)
	GPIOB->MODER |= (BIT0|BIT1);  // Select analog mode for PB0 (pin 14 of LQFP32 package)
	GPIOB->MODER |= (BIT2|BIT3);  // Select analog mode for PB1 (pin 15 of LQFP32 package)

	initADC();
	
	// Configure the pin used to measure period
	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);
	
	// // Configure the pin connected to the pushbutton as input

	// GPIOA->MODER &= ~(BIT28 | BIT29); // Make pin PA14 input
	// // Activate pull up for pin PA8:
	// GPIOA->PUPDR |= BIT28; 
	// GPIOA->PUPDR &= ~(BIT29);

	GPIOA->MODER &= ~(BIT2 | BIT3); // Make pin PA1 input
	// Activate pull up for pin PA1:
	GPIOA->PUPDR |= BIT2; 
	GPIOA->PUPDR &= ~(BIT3);
	
	// Configure some pins as outputs:
	// Make pins PB3 to PB7 outputs (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
    GPIOB->MODER = (GPIOB->MODER & ~(BIT6|BIT7)) | BIT6;    // PB3
	GPIOB->OTYPER &= ~BIT3; // Push-pull
    GPIOB->MODER = (GPIOB->MODER & ~(BIT8|BIT9)) | BIT8;    // PB4
	GPIOB->OTYPER &= ~BIT4; // Push-pull
    GPIOB->MODER = (GPIOB->MODER & ~(BIT10|BIT11)) | BIT10; // PB5
	GPIOB->OTYPER &= ~BIT5; // Push-pull
    GPIOB->MODER = (GPIOB->MODER & ~(BIT12|BIT13)) | BIT12; // PB6
	GPIOB->OTYPER &= ~BIT6; // Push-pull
    GPIOB->MODER = (GPIOB->MODER & ~(BIT14|BIT15)) | BIT14;  // PB7
	GPIOB->OTYPER &= ~BIT7; // Push-pull
	
	// Set up servo PWM output pins
    GPIOA->MODER = (GPIOA->MODER & ~(BIT22|BIT23)) | BIT22; // Make pin PA11 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER |= BIT11; // Open-drain
    GPIOA->MODER = (GPIOA->MODER & ~(BIT24|BIT25)) | BIT24; // Make pin PA12 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER |= BIT12; // Open-drain

	// Set up JDY40 output pins
	GPIOA->MODER = (GPIOA->MODER & ~(BIT26|BIT27)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	// Set up LED Debugging output pins
	GPIOA->MODER = (GPIOA->MODER & ~(BIT12|BIT13)) | BIT12; // Make pin PA6 output
	GPIOA->OTYPER &= ~BIT6; // Push-pull
	GPIOA->MODER = (GPIOA->MODER & ~(BIT14|BIT15)) | BIT14; // Make pin PA7 output
	GPIOA->OTYPER &= ~BIT7; // Push-pull

	// Set up timers
	RCC->APB1ENR |= BIT0;  // turn on clock for timer2 (UM: page 177)
	TIM2->ARR = F_CPU/DEF_F-1;
	NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC
	TIM2->CR1 |= BIT4;      // Downcounting    
	TIM2->CR1 |= BIT7;      // ARPE enable    
	TIM2->DIER |= BIT0;     // enable update event (reload event) interrupt 
	TIM2->CR1 |= BIT0;      // enable counting    


	__enable_irq();
}


long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PA8!=0) // Wait for square wave to be 0
	{
		//eputs("PA8!=0 While Loop in GetPeriod\r\n");
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PA8==0) // Wait for square wave to be 1
	{
		//eputs("PA8==0 While Loop in GetPeriod\r\n");
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PA8!=0) // Wait for square wave to be 0
		{
			//eputs("for loop PA8!=0 While Loop in GetPeriod\r\n");
			if(SysTick->CTRL & BIT16) return 0;
		}
		while (PA8==0) // Wait for square wave to be 1
		{
			//eputs("for loop PA8==0 While Loop in GetPeriod\r\n");
			if(SysTick->CTRL & BIT16) return 0;
		}
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	return 0xffffff-SysTick->VAL;
}

void PrintNumber(long int val, int Base, int digits)
{ 
	char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	eputs(&buff[j+1]);
}

/****************************************************************************************************************
RADIO FUNCTIONS
****************************************************************************************************************/

void SendATCommand (char * s)
{
	char buff[40];
	// printf("Command: %s", s);
	eputs("Command: ");
	eputs(s);
	eputs("\r\n");
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	// printf("Response: %s", buff);
	eputs("Response: ");
	eputs(buff);
	eputs("\r\n");
}

void ReceptionOff (void)
{
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	while (ReceivedBytes2()>0) egetc2(); // Clear FIFO
}

/***********************************************************************
 * Process ALL incoming radio data from UART2
 *    - If we see a '!' we parse a 6-char command (e.g., "Ns1515") 
 *      and call ManualControl().
 *    - If we see an '@' we respond with frequency data.
 *    - Otherwise, we ignore or handle as needed.
 **********************************************************************/
void processRadioData(char *buff, RemoteCommand *currentCmd)
{
	
    // Read until the FIFO is empty.
    while (ReceivedBytes2() > 0)
    {
        char c = egetc2();

        if (c == '!')
        {
            // We expect the next 6 characters to be the joystick/buttons, e.g. "Ns1515".
            // Read 6 chars (plus 1 for '\0').
            if (egets2(buff, 7) > 0)
            {
                eputs("Received command: ");
                eputs(buff);
                eputs("\r\n");

                // Try to parse into our RemoteCommand struct, then do manual control
                if (parseRemoteCommand(buff, currentCmd))
                {
                    ManualControl(currentCmd); // <--- your existing manual-control logic
                    
                    if (currentCmd->b1 == START)
                    {
                        *mode_flag = 1;
                    }
                    else if (currentCmd->b1 == SELECT)
                    {
                        *mode_flag = 0;
                    }
                    else if (currentCmd->b1 ==START_SELECT)
                    {
                        *mode_flag = 3;
                    }                    
                }
                else
                {
                    eputs("Error: unable to parse command!\r\n");
                }
            }
        }
       else if (c == '@')   {
    		// Master is requesting frequency data and coin count.
    		// Format as "F:xxxxx,C:yyy" where frequency is 5 digits and coin_count is 3 digits.
    		
        	long int freq = frequency *10000;  // recalc if needed
    		freq = freq / reference_frequency;
    		
    		sprintf(buff, "%05ld%02d", freq, coin_count);
    		// The radio often needs a small delay
    		waitms(5);
    		eputs2(buff);   // Send back the formatted data string
		}
        else {
            //eputs("NO DATA\r\n");
        }
    }
}


/* parseRemoteCommand() parses a string of the form:
     "LX=%d,LY=%d,RX=%d,RY=%d,B6=%d,B7=%d"
   Returns 1 on success, 0 on failure.
*/
//int parseRemoteCommand(char *str, RemoteCommand *cmd) 
//{
//    int parsed = sscanf(str, "LX=%d,LY=%d,RX=%d,RY=%d,B6=%d,B7=%d",
//                        &cmd->lx, &cmd->ly, &cmd->rx, &cmd->ry,
//                        &cmd->b6, &cmd->b7);
//    return (parsed == 6);
//}

//Checks if SELECT or START are Pressed
//If true, starts or stops automode based on value of autoflag and resets robot to starting position
//Returns 1 if moving to Manual mode, otherwise returns 0
int automode_toggle_check (char button)
{
	//char * data;
	eputs("automode_toggle_check\r\n");
	if (button == 's') // If SELECT is pressed, turn off automode
	{	
		eputs("Turn off Automode\r\n");
		*mode_flag = 0;

		leftWheelStop();
		rightWheelStop();
		resetArm();

		return 1;
	}
	else if(button == 'S') // If START is pressed, turn on automode
	{
		eputs("Turn on Automode\r\n");
		*mode_flag = 1;
		return 0;
	}
	else
	{
		eputs("Automode no effect\r\n");
		return 0;
	}

}
/****************************************************************************************************************
MOVEMENT FUNCTIONS
****************************************************************************************************************/

// Functions for controlling the wheels
// Change the wiring to match these functions
void leftWheelForward(void)
{
	PB3_1;
	PB4_0;
}

void leftWheelStop(void)
{
	PB3_0;
	PB4_0;
}

void leftWheelBackward(void)
{
	PB3_0;
	PB4_1;
}

void rightWheelForward(void)
{
	PB5_1;
	PB6_0;
}

void rightWheelStop(void)
{
	PB5_0;
	PB6_0;
}

void rightWheelBackward(void)
{
	PB5_0;
	PB6_1;
}
void turnAround(void)
{
	eputs("turnAround\r\n");
    // Change this number to make sure the robot turns 90 degrees in this amount of time
    int turning_time_ms = 200;
    
    // Go backwards for some time
    leftWheelBackward();
    rightWheelBackward();
    
    // This may need adjustment
    waitms(500);
    
    // The three lines of code here may not be necessary
    // leftWheelStop();
    // righttWheelStop();
    // waitms(100);
    
    // Turn clockwise 90 degrees
    leftWheelForward();
    rightWheelBackward();
    waitms(turning_time_ms);
    
    // PWM_Counter is constantly changing from 1 to 2000 in Timer2; use it for pseudorandom numbers
    // Turn clockwise from 0 to 180 degrees more
    waitms((turning_time_ms * PWM_Counter)/1000);

    // The three lines of code here may not be necessary
    // leftWheelStop();
    // righttWheelStop();
    // waitms(100);
}

void turn90DegreesCW(void){
	eputs("turn90DegreesCW\r\n");

    // Turn clockwise 90 degrees
    leftWheelBackward();
    rightWheelForward();
    waitms(600);

    //Stop
    leftWheelStop();
    rightWheelStop();
}

void turn90DegreesCCW(void){
	eputs("turn90DegreesCCW\r\n");

    // Turn counter clockwise 90 degrees
    leftWheelForward();
    rightWheelBackward();
    waitms(600);
	

    //Stop
    leftWheelStop();
    rightWheelStop();
}

//Inch robot forward slightly
void aLittleForward(void)
{
	leftWheelForward();
	rightWheelForward();
	waitms(100);
	leftWheelStop();
	rightWheelStop();
}
void aLittleBackward(void)
{
	leftWheelBackward();
	rightWheelBackward();
	waitms(100);
	leftWheelStop();
	rightWheelStop();
}
void autodrive(RemoteCommand *currentCommand)
{
    char buff[16];
    //RemoteCommand currentCommand;

    eputs("autodrive\r\n");

    //processRadioData(buff, &currentCommand);

    // If a toggle command (SELECT: 's' or START: 'S') was received, handle it.
    if (currentCommand->b1 == SELECT)
    {
        *mode_flag = 0;
        coin_count = 0;
        return; // Exit autonomous mode and reset coin count to 0
    }

    //check if pause command was recieved
    if(currentCommand->b2 == TRIANGLE)
    {
        *mode_flag = 2;
        return;
    }

    // Drive forward.
    leftWheelForward();
    rightWheelForward();

    // Update frequency measurement.
    //frequency = (F_CPU * 60) / GetPeriod(60);

    // If metal is detected, pick up the coin.
    if (detectMetal())
    {
    	eputs("while(detectMetal())\r\n");
        collectCoin();
    }

    // If the perimeter is detected, perform a turn-around.
    if (detectPerimeter())
    {
    	eputs("if(detectPerimeter())\r\n");
        turnAround();
    }

    // (Any frequency requests are handled within processRadioData.)
    // If enough coins are collected, end automode.
    if (coin_count == 20)
    {
        //processRadioData(buff, &currentCommand); // Check for any pending commands.
        *mode_flag = 0; // Exit autonomous mode.
        coin_count = 0;
    }

    waitms(50);

    return;
}

//Displays countdown and lets user try to pick up as many coins as they can in a given time
//Alternatively: Has set amount of coins to pick up and lets user try to pick up as many as they can in given time
//If we want to be able to count coins we will need a load cell
void gameMode(void)
{
    eputs("gameMode\r\n");
}

/*************************************************************************************
 * MANUAL MODE
 * This function processes a decoded remote command (RemoteCommand structure)
 * and drives the robot accordingly in manual mode.
 * It adjusts the speed scaling using L1 and R1, toggles the electromagnet with the
 * X button, and calculates forward/reverse speed and turning based on the left 
 * joystick's Y and X axes respectively.
 * The thresholds for the deadzone are set using LEVEL4 and LEVEL5.
 *************************************************************************************/
    
    // ----- Control electromagnet servo (raise/lower arm) using DPAD (buttons from set 1) -----
    // Define target servo positions for the electromagnet arm.
void ManualControl(RemoteCommand *command) {
    // ----- Static variables -----
    static int speedFactor = 50;    // Initial speed scaling factor (percentage)
    static int electromagnetOn = 0;  // 0: off, 1: on.
    static int firstRun = 1;         // One-time initialization flag

    // Initialize electromagnet state on first run.
    if (firstRun) {
        PB7_0; // Turn electromagnet OFF.
        electromagnetOn = 0;
        firstRun = 0;
        eputs("Initial setup: Electromagnet OFF.\r\n");
    }

    // ----- Adjust speed factor using L1 and R1 (button set 2) -----
    //if (command->b2 == L1) {       
    //   speedFactor -= 10;
    //   eputs("L1 pressed: Decreasing speed factor.\r\n");
    //}
    //if (command->b2 == R1) {       
    //   speedFactor += 10;
    //   eputs("R1 pressed: Increasing speed factor.\r\n");
    //}
    //if (speedFactor < 0)   speedFactor = 0;
    //if (speedFactor > 100) speedFactor = 100;
    
    // ----- Control electromagnet on/off (using X and O from button set 2) -----
    if (command->b2 == X_BUTTON) {
        PB7_1;        // Turn electromagnet ON.
        electromagnetOn = 1;
        eputs("X pressed: Turning electromagnet ON.\r\n");
    }
    if (command->b2 == O_BUTTON) {
        PB7_0;        // Turn electromagnet OFF.
        electromagnetOn = 0;
        eputs("O pressed: Turning electromagnet OFF.\r\n");
    }
    
    // ----- Adjust electromagnet arm position using DPAD (button set 1) -----
    // Vertical movement (up/down remains the same)
    if (command->b1 == DPAD_UP) {
        if (ISR_pwm1 > MIN_ISR_PWM1) {
            ISR_pwm1 -= VERTICAL_STEP;
            waitms(VERTICAL_DELAY_MS);
            eputs("DPAD UP pressed: Moving electromagnet up.\r\n");
        }
    }
    if (command->b1 == DPAD_DOWN) {
        if (ISR_pwm1 < MAX_ISR_PWM1) {
            ISR_pwm1 += VERTICAL_STEP;
            waitms(VERTICAL_DELAY_MS);
            eputs("DPAD DOWN pressed: Moving electromagnet down.\r\n");
        }
    }
    // Inverted horizontal movement: 
    // DPAD_LEFT now increases ISR_pwm2 (moves right) and DPAD_RIGHT decreases ISR_pwm2 (moves left)
    if (command->b1 == DPAD_LEFT) {
        if (ISR_pwm2 < MAX_ISR_PWM2) {
            ISR_pwm2 += HORIZONTAL_STEP;
            waitms(HORIZONTAL_DELAY_MS);
            eputs("DPAD LEFT pressed: Moving electromagnet right (inverted).\r\n");
        }
    }
    if (command->b1 == DPAD_RIGHT) {
        if (ISR_pwm2 > MIN_ISR_PWM2) {
            ISR_pwm2 -= HORIZONTAL_STEP;
            waitms(HORIZONTAL_DELAY_MS);
            eputs("DPAD RIGHT pressed: Moving electromagnet left (inverted).\r\n");
        }
    }
    
    // ----- Compute wheel speeds from joystick positions -----
    // Use the global mapValue() function (declared elsewhere) to scale joystick inputs.
    int leftSpeed = 0;
    if (command->ry < LEVEL4) {
         leftSpeed = mapValue(command->ry, 0, LEVEL4, MAX_SPEED, 0);
    } else if (command->ry > LEVEL5) {
         leftSpeed = mapValue(command->ry, LEVEL5, 255, 0, -MAX_SPEED);
    }else {
	 leftSpeed = 0;   
    }
	
    leftSpeed = (leftSpeed * speedFactor) / 100;
    
    int rightSpeed = 0;
    if (command->ly < LEVEL4) {
         rightSpeed = mapValue(command->ly, 0, LEVEL4, MAX_SPEED, 0);
    } else if (command->ly > LEVEL5) {
         rightSpeed = mapValue(command->ly, LEVEL5, 255, 0, -MAX_SPEED);
    }else {
	 rightSpeed = 0;   
    }
    rightSpeed = (rightSpeed * speedFactor) / 100;

    // ----- Debug prints -----
    eputs("ManualControl: leftSpeed=");
    PrintNumber(leftSpeed, 10, 3);
    eputs(", rightSpeed=");
    PrintNumber(rightSpeed, 10, 3);
    eputs(", speedFactor=");
    PrintNumber(speedFactor, 10, 3);
    eputs("\r\n");
    
    // ----- Command the wheels -----
    if (leftSpeed > SPEED_THRESHOLD)
         leftWheelForward();
    else if (leftSpeed < -SPEED_THRESHOLD)
         leftWheelBackward();
    else
         leftWheelStop();
    
    if (rightSpeed > SPEED_THRESHOLD)
         rightWheelForward();
    else if (rightSpeed < -SPEED_THRESHOLD)
         rightWheelBackward();
    else
         rightWheelStop();
  
    //if (decodeJoystick(command->ly, 0) > LEVEL5)
      //   leftWheelForward();
    //else if (decodeJoystick(command->ly, 0) < LEVEL4)
      //   leftWheelBackward();
    //else
      //   leftWheelStop();
    
    //if (decodeJoystick(command->ry, 0) > LEVEL5)
      //   rightWheelForward();
    //else if (decodeJoystick(command->ry, 0) < LEVEL4)
      //   rightWheelBackward();
    //else
      //   rightWheelStop();



    //----- More Driving Functions -----
    //Press L3 to turn robot 90 degrees counter clockwise
    if(command->b1 == L3)
    {
        turn90DegreesCCW();
    }

    //Press L3 to turn robot 90 degrees counter clockwise
    if(command->b1 == R3)
    {
        turn90DegreesCW();
    }

    //Press R1 to inch the robot backward	
    if(command->b2 == R1)
    {
	    aLittleBackward();
    }	    

    //Press R2 to inch the robot forward	
    if(command->b2 == R2)
    {
	    aLittleForward();
    }	 
	
    //----- Collect Coin -----
    if(command->b2 == L2)     
    {
        collectCoin();
    }

    //


}


/****************************************************************************************************************
ELECTROMAGNETIC FUNCTIONS
****************************************************************************************************************/

// Function for detecting metal
// Returns 1 if metal is detected, else 0
int detectMetal(void)
{
	int threshold = 128;
    int voltage_threshold = 1000;
	
	eputs("detectMetal\r\n");
	//long int count;

	//count = GetPeriod(60);

	//frequency = (F_CPU * 60) / count;

	//printf("Count = %6d\r\n", count);
	
	voltage[0] = (readADC(ADC_CHSELR_CHSEL8) * 33000) / 0xfff;
	voltage[1] = (readADC(ADC_CHSELR_CHSEL9) * 33000) / 0xfff;

	// if (count > reference_count + threshold)
	// {
	// 	PA6_0;
	// 	return 1;
	// }

    if (voltage[0] < reference_voltage[0] - voltage_threshold || voltage[1] < reference_voltage[1] - voltage_threshold)
    {
        PA6_0;
        return 1;
    }
	
	PA6_1;
	return 0;
}

// Function for detecting the perimeter
// Returns 1 if perimeter is reached, else 0
int detectPerimeter(void)
{
	int voltage_threshold = 2500;
	
	eputs("detectPerimeter\r\n");
	voltage[0] = (readADC(ADC_CHSELR_CHSEL8) * 33000) / 0xfff;
	voltage[1] = (readADC(ADC_CHSELR_CHSEL9) * 33000) / 0xfff;
	
	if (voltage[0] > reference_voltage[0] + voltage_threshold || voltage[1] > reference_voltage[1] + voltage_threshold)
	{
		PA7_0;
		return 1;
	}

	PA7_1;
	return 0;
}

// Function for picking up a coin
void collectCoin(void)
{
	eputs("collectCoin\r\n");
	// Move robot backwards (if necessary)
	leftWheelBackward();
	rightWheelBackward();
	//if(automode_toggle_check()){
		//return;
	//}
	waitms(250);


	leftWheelStop();
	rightWheelStop();
	//if(automode_toggle_check()){
		//return;
	//}
	waitms(500);

	// Move arm down from starting position
	while (ISR_pwm2 < 255)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm2++;
		waitms(5);
	}
	while (ISR_pwm1 < 255)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm1++;
		waitms(5);
	}

	// Turn on electromagnet
	PB7_1;
	waitms(100);

	// Sweep left and right
	while (ISR_pwm2 > 140)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm2--;
		waitms(10);
	}

	waitms(100);

	while (ISR_pwm2 < 160)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm2++;
		waitms(10);
	}

	waitms(100);

	// Move arm above collection box
	while (ISR_pwm1 > 130)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm1--;
		waitms(5);

	}
	
	waitms(100);
	
	while (ISR_pwm2 > 110)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm2--;
		waitms(5);
	}

	// Move arm above collection box
	while (ISR_pwm1 < 170)
	{
		//if(automode_toggle_check()){
			//return;
		//}
		ISR_pwm1++;
		waitms(5);

	}

	// Turn off electromagnet
	PB7_0;
	waitms(1000);
	
	while (ISR_pwm1 > 60)
	{
		ISR_pwm1--;
		waitms(5);
	}

	//put coin sound function here 

	
	// Increment the coin counter when a coin is collected
    	coin_count++;
	
	// Return to starting position
	resetArm();
	
	waitms(1000);
}

void resetArm(void)
{
	// Turn off electromagnet
	PB7_0;
	waitms(100);

	// Move arm to starting position
	while (ISR_pwm2 < 255)
	{
		ISR_pwm2++;
		waitms(5);
	}
	while (ISR_pwm1 > 60)
	{
		ISR_pwm1--;
		waitms(5);
	}
}

/****************************************************************************************************************
MAIN
****************************************************************************************************************/

int main(void)
{
    Hardware_Init();
    initUART2(9600);

    // Just so your pointer mode_flag is valid:
    static int mode_value = 0;  // 0 = manual, 1 = auto, 2 = paused
    mode_flag = &mode_value;
    
    char buff[16];
    RemoteCommand currentCommand; 

    // Clear screen and print startup messages.
    eputs("\x1b[2J\x1b[1;1H");
    eputs("\r\nSTM32L051 multi I/O example.\r\n");
    eputs("Starting in MANUAL MODE.\r\n");

    ReceptionOff();

    // Optional: send some AT commands.
    SendATCommand("AT+VER\r\n");
    SendATCommand("AT+BAUD\r\n");
    SendATCommand("AT+RFID\r\n");
    SendATCommand("AT+DVID\r\n");
    SendATCommand("AT+RFC\r\n");
    SendATCommand("AT+POWE\r\n");
    SendATCommand("AT+CLSS\r\n");
    SendATCommand("AT+DVID0A13\r\n");  

    // Initialize wheels and electromagnet
    PB3_0;
    PB4_0;
    PB5_0;
    PB6_0;
    PB7_0;
    PA6_1;
    PA7_1;

    // Measure reference voltages.
    reference_voltage[0] = (readADC(ADC_CHSELR_CHSEL8) * 33000) / 0xfff;
    reference_voltage[1] = (readADC(ADC_CHSELR_CHSEL9) * 33000) / 0xfff;

	while (1)
	{
		reference_count = GetPeriod(60);
	
		if (reference_count > 0)
		{
			break;
		}
	
		eputs("NO SIGNAL                     \r\n");
	}

	reference_frequency = (F_CPU * 60) / reference_count;
    // Main loop
    while (1)
    {
    	count = GetPeriod(60);
	    
        if (count > 0)
        {
            frequency = (F_CPU * 60) / count;
	}
        processRadioData(buff, &currentCommand);

        // If robot is paused and unpause command is received.
        if (*mode_flag == 2 && currentCommand.b2 == SQUARE)
        {
            *mode_flag = 1;
        }

        // Run autonomous routine if flag is set.
        if (*mode_flag == 1)
        {
            eputs("AUTONOMOUS MODE ACTIVE.\r\n");
            autodrive(&currentCommand);
            if (*mode_flag == 0)
            {
                eputs("Returning to MANUAL MODE.\r\n");
                leftWheelStop();
                rightWheelStop();
            }
        }
        else if (*mode_flag == 3)
        {
            eputs("GAME MODE ACTIVE.\r\n");
            gameMode();            
        }

        waitms(2);
    }

    return 0;
}
