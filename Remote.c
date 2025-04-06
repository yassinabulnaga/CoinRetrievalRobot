#include <XC.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <string.h>

// =========== CONFIGURATION BITS ===========
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYSCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator off

// =========== DEFINES ===========
#define SYSCLK          40000000L
#define PBCLK           SYSCLK
#define CHARS_PER_LINE  16
#define DEF_FREQ 2205L
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

// Macro to convert desired baud rate to BRG value
#define Baud2BRG(desired_baud) ( (PBCLK / (16*desired_baud))-1 )

// =========== LCD PIN DEFINITIONS ===========
#define LCD_RS          LATBbits.LATB3
#define LCD_RS_ENABLE   TRISBbits.TRISB3

#define LCD_E           LATAbits.LATA2
#define LCD_E_ENABLE    TRISAbits.TRISA2

#define LCD_D4          LATAbits.LATA3
#define LCD_D4_ENABLE   TRISAbits.TRISA3

#define LCD_D5          LATBbits.LATB4
#define LCD_D5_ENABLE   TRISBbits.TRISB4

#define LCD_D6          LATAbits.LATA4
#define LCD_D6_ENABLE   TRISAbits.TRISA4

#define LCD_D7          LATBbits.LATB5
#define LCD_D7_ENABLE   TRISBbits.TRISB5

// =========== SPEAKER PIN DEFINITIONS =======
#define SPEAKER         LATBbits.LATB12
#define SPEAKER_TRIS    TRISBbits.TRISB12

#define PIN_PERIOD PORTAbits.RA0

// =========== PS2 PIN DEFINITIONS ===========
#define PS2_DATA        PORTBbits.RB0     // Input
#define PS2_DATA_TRIS   TRISBbits.TRISB0
#define PS2_CMD         LATBbits.LATB1    // Output
#define PS2_CMD_TRIS    TRISBbits.TRISB1
#define PS2_ATT         LATBbits.LATB2    // Output
#define PS2_ATT_TRIS    TRISBbits.TRISB2
#define PS2_CLK         LATBbits.LATB10   // Output
#define PS2_CLK_TRIS    TRISBbits.TRISB10

//  =========== PS2 Button DEFINITIONS ===========
#define LEVEL0 0
#define LEVEL1 28
#define LEVEL2 55
#define LEVEL3 82
#define LEVEL4 110
#define LEVEL5 146
#define LEVEL6 174
#define LEVEL7 201
#define LEVEL8 228
#define LEVEL9 256

// Button Values 
#define NONE 255 //Verified
#define SQUARE 127 // verified
#define O_BUTTON 223 // verified
#define TRIANGLE 239 // verified
#define X_BUTTON 191 // verified
#define R1 247 // verified
#define R2 253 // verified
#define L1 251 // verified
#define L2 254 // verified

#define START 247 // verified
#define SELECT 254 // verified
#define L3 253 // verified
#define R3 251 // verified
#define DPAD_UP 239 // verified
#define DPAD_DOWN 191 // verified
#define DPAD_LEFT 127 // verified
#define DPAD_RIGHT 223 // verified

// =========== FUNCTION PROTOTYPES ===========
void Timer4us(unsigned char t);
void waitms(unsigned int ms);
void waitus(unsigned int us);
void LCD_pulse(void);
void LCD_byte(unsigned char x);
void WriteData(unsigned char x);
void WriteCommand(unsigned char x);
void LCD_4BIT(void);
void LCDprint(char * string, unsigned char line, unsigned char clear);
void requestSlaveData(void);

// === ADDED/MODIFIED ===
void beepSpeaker(void);
int calcSignalStrength(int frequency);

void UART2Configure(int baud_rate);
int _mon_getc(int canblock);

void PS2_Init(void);
unsigned char PS2_TransferByte(unsigned char outByte);
void PS2_ReadData(unsigned char *d);

// =========== GLOBALS (if you need them) ===========
int global_frequency = 0;
int global_coinCount = 0;

volatile unsigned int T2_overflow = 0;

// =========== LCD ROUTINES ===========
void Timer4us(unsigned char t) 
{
    T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
    while(t >= 100)
    {
        t -= 100;
        TMR4 = 0;
        while(TMR4 < (SYSCLK/10000L));
    }
    while(t >= 10)
    {
        t -= 10;
        TMR4 = 0;
        while(TMR4 < (SYSCLK/100000L));
    }
    while(t > 0)
    {
        t--;
        TMR4 = 0;
        while(TMR4 < (SYSCLK/1000000L));
    }
    T4CONCLR = 0x8000;
}

void waitms(unsigned int ms)
{
    unsigned int j;
    for(j = 0; j < ms; j++)
    {
        Timer4us(250);
        Timer4us(250);
        Timer4us(250);
        Timer4us(250);
    }
}

void waitus(unsigned int us)
{
    while(us--)
        Timer4us(1);
}

void LCD_pulse(void)
{
    LCD_E = 1;
    Timer4us(40);
    LCD_E = 0;
}

void LCD_byte(unsigned char x)
{
    LCD_D7 = (x & 0x80) ? 1 : 0;
    LCD_D6 = (x & 0x40) ? 1 : 0;
    LCD_D5 = (x & 0x20) ? 1 : 0;
    LCD_D4 = (x & 0x10) ? 1 : 0;
    LCD_pulse();
    Timer4us(40);
    LCD_D7 = (x & 0x08) ? 1 : 0;
    LCD_D6 = (x & 0x04) ? 1 : 0;
    LCD_D5 = (x & 0x02) ? 1 : 0;
    LCD_D4 = (x & 0x01) ? 1 : 0;
    LCD_pulse();
}

void WriteData(unsigned char x)
{
    LCD_RS = 1;
    LCD_byte(x);
    waitms(2);
}

void WriteCommand(unsigned char x)
{
    LCD_RS = 0;
    LCD_byte(x);
    waitms(5);
}

void LCD_4BIT(void)
{
    LCD_RS_ENABLE = 0;
    LCD_E_ENABLE  = 0;
    LCD_D4_ENABLE = 0;
    LCD_D5_ENABLE = 0;
    LCD_D6_ENABLE = 0;
    LCD_D7_ENABLE = 0;

    LCD_E = 0;
    waitms(20);

    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32);

    WriteCommand(0x28);
    WriteCommand(0x0c);
    WriteCommand(0x01);
    waitms(20);
}

void LCDprint(char * string, unsigned char line, unsigned char clear)
{
    int j;
    if(line == 2)
        WriteCommand(0xc0);
    else
        WriteCommand(0x80);
    waitms(5);
    for(j = 0; string[j] != 0; j++)
    {
        WriteData(string[j]);
    }
    if(clear)
    {
        for(; j < CHARS_PER_LINE; j++)
            WriteData(' ');
    }
}


// ========== JDY-40 ROUTINES =============

void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = c;          // echo
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

//Functions from Set up Two Timers

void uart_putc (unsigned char c)
{
    while( U2STAbits.UTXBF); // wait while TX buffer full
    U2TXREG = c; // send single character to transmit buffer
}

void uart_puts (char * buff)
{
	while (*buff)
	{
		uart_putc(*buff);
		buff++;
	}
}

unsigned char uart_getc (void)
{
	unsigned char c;
	
	while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	c=U2RXREG;
	return c;
}

void SetupTimer1 (void)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/DEF_FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // Pre-scaler: 1
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

volatile unsigned int Tick_Counter=0;
volatile unsigned char Second_Flag=1;

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
    // Calculate the signal strength (clamp to 0..100)
    int strength = calcSignalStrength(global_frequency);
    if(strength > 100)
        strength = 100;
    else if(strength < 0)
        strength = 0;
    
    // Map strength to a beep repetition rate.
    // Here, beepFreq will vary from 5 (for low strength) to 55 (for high strength)
    int beepMinFreq = 5; 
    int beepMaxFreq = 30;
    int beepFreq = beepMinFreq + (beepMaxFreq - beepMinFreq) * strength / 50;
    
    // Clear the timer interrupt flag
    IFS0CLR = _IFS0_T1IF_MASK; 
    
    // Use a state machine to control the on/off period of the beep (without changing the speaker pitch).
    // The idea is: a fixed “on” period (250 ticks) and a variable “off” period (50000/beepFreq ticks).
    if(Second_Flag == 0)
    {
        SPEAKER = 0;      // Turn the speaker off during the off period.
        Tick_Counter++;
        // Use >= instead of == so that any overshoot still resets the counter.
        if(Tick_Counter >= (50000 / beepFreq))
        {
            Tick_Counter = 0;
            Second_Flag = 1;
        }
    }
    else // Second_Flag == 1
    {
        SPEAKER = !SPEAKER;   // Toggle the speaker (this produces the constant pitch square wave)
        Tick_Counter++;
        if(Tick_Counter >= 250)
        {
            Tick_Counter = 0;
            Second_Flag = 0;
        }
    }
}

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}

/////////////////////////////////////////////////////////
// UART1 functions used to communicate with the JDY40  //
/////////////////////////////////////////////////////////

// TXD1 is in pin 26
// RXD1 is in pin 24

int UART1Configure(int desired_baud)
{
	int actual_baud;

    // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
    // 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2

    ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB13 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
    U1RXRbits.U1RXR = 3; // SET U1RX to RB13

    // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
    // RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7

    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
	
    U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U1STA = 0x1400;     // enable TX and RX
    U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    actual_baud = SYSCLK / (16 * (U1BRG+1));

    U1MODESET = 0x8000;     // enable UART1

    return actual_baud;
}

void putc1 (char c)
{
	while( U1STAbits.UTXBF);   // wait while TX buffer full
	U1TXREG = c;               // send single character to transmit buffer
}
 
int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U1STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

// Copied from here: https://forum.microchip.com/s/topic/a5C3l000000MRVAEA4/t335776
void delayus(uint16_t uiuSec)
{
    uint32_t ulEnd, ulStart;
    ulStart = _CP0_GET_COUNT();
    ulEnd = ulStart + (SYSCLK / 2000000) * uiuSec;
    if(ulEnd > ulStart)
        while(_CP0_GET_COUNT() < ulEnd);
    else
        while((_CP0_GET_COUNT() > ulStart) || (_CP0_GET_COUNT() < ulEnd));
}

unsigned int SerialReceive1_timeout(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    int timeout_cnt;
 
    while(num_char < max_size)
    {
    	timeout_cnt=0;
    	while(1)
    	{
	        if(U1STAbits.URXDA) // check if data is available in RX buffer
	        {
	        	*buffer = U1RXREG; // copy RX buffer into *buffer pointer
	        	break;
	        }
	        if(++timeout_cnt==100) // 100 * 100us = 10 ms
	        {
	        	*buffer = '\n';
	        	break;
	        }
	        delayus(100);
	    }
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void ClearFIFO (void)
{
	unsigned char c;
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	while(U1STAbits.URXDA) c=U1RXREG;
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1(s);
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	SerialReceive1(buff, sizeof(buff)-1);
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	delayms(10);
	printf("Response: %s\n", buff);
}

void ReceptionOff (void)
{
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing.
	delayms(10);
	ClearFIFO();
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
}


// ================ SPEAKER AND JDY-40 RECEIVING DATA ROUTINES =================


int calcSignalStrength(int frequency)
{

	int signal_strength = 10000 - frequency;
	
	if(signal_strength < 0) {
		return 0;
	} else if (signal_strength > 100) {
		return 100;
	}
	
    return (signal_strength*100)/30;
}

void beepSpeaker(void)
{
    //if(Second_Flag == 0){
    //	SPEAKER = 0;
    //}
    //else if(Second_Flag == 1){
    //	SPEAKER = 1;
    //}
    // Compute signal strength based on the global frequency
    //int strength = calcSignalStrength(global_frequency);
    //if (strength == 0)
    //    return;

    // Determine beep frequency based on strength (example linear mapping)
    //int beepMinFreq = 1; 
    //int beepMaxFreq = 10;
    //int beepFreq = beepMinFreq + (beepMaxFreq - beepMinFreq) * strength / 100;

    // Calculate timer ticks for a half period.
    // The Timer1 interrupt will fire every half period so that toggling the pin creates a square wave.
    // (SYSCLK / (2 * beepFreq)) gives the number of ticks for a half period.
    //unsigned int halfPeriodTicks = (SYSCLK / (2 * beepFreq)) - 1;

    // Reconfigure Timer1 for the desired half period.
	//if(Tick_Counter==22050/beepFreq)
	//{
	//	Tick_Counter=0;
		
	//	__builtin_disable_interrupts();
	//}	
    //__builtin_disable_interrupts();
    //PR1 = halfPeriodTicks;    // Set period for half period timing
    //TMR1 = 0;                 // Reset timer count
    //T1CONbits.TCKPS = 0;      // Pre-scaler: 1
    //T1CONbits.TCS = 0;        // Internal clock (PBCLK)
    //T1CONbits.ON = 1;         // Turn on Timer1 so that its interrupt will toggle the speaker pin
    //__builtin_enable_interrupts();

    // Let the beep sound for ~100ms.
    // (During this time, Timer1 interrupts will toggle SPEAKER in the ISR.)
    //  waitms(100);

    // After 100ms, disable Timer1 and turn the speaker off.
    //__builtin_disable_interrupts();
    //T1CONbits.ON = 0;         // Stop Timer1
    //SPEAKER = 0;              // Ensure speaker is low (off)
    //__builtin_enable_interrupts();
}

void requestSlaveData(void)
{
    char buff[20];   // Buffer to hold the slave's response
    int timeout_cnt = 0;
    
    // Clear the receive FIFO so we get a fresh reply from the slave
     if(U1STAbits.URXDA)
         SerialReceive1_timeout(buff, sizeof(buff)-1);
	
        
	
    // Wait for a response with a timeout up to ~50ms
    timeout_cnt = 0;
    while(1)
    {
        if(U1STAbits.URXDA) break;     // Data has arrived
        if(++timeout_cnt > 100000) break; // Timeout after ~50ms
        delayus(100);                  // 100us delay per iteration
    }
    
    // If data is available, read it
    if(U1STAbits.URXDA)
    {
        SerialReceive1_timeout(buff, sizeof(buff)-1);
        if(strlen(buff) > 0)
        {
            // Debug print of the raw data received from the slave
            printf("Slave says: %s\r\n", buff);
            
            // === ADDED/MODIFIED ===
            // We now expect exactly 7 characters: first 5 for freq, last 2 for coin count
            // e.g. "1234512"
            if(strlen(buff) == 7)
            {
                char freqStr[6];
                char coinStr[3];
                
                strncpy(freqStr, buff, 5);
                freqStr[5] = '\0';
                strncpy(coinStr, &buff[5], 2);
                coinStr[2] = '\0';
                
                global_frequency = atoi(freqStr);
                global_coinCount = atoi(coinStr);
                
                // Call the speaker beep function based on frequency
                //beepSpeaker(global_frequency);
                
                // Display signal strength on LCD (line 1)
                char line1[20];
                int strength = calcSignalStrength(global_frequency);
                if (strength > 100){
                	strength = 100;
                }	
                sprintf(line1, "Strength: %d%%", strength);
                LCDprint(line1, 1, 1);

                // Display coin count on LCD (line 2)
                char line2[20];
                sprintf(line2, "Coin Count: %d", global_coinCount);
                LCDprint(line2, 2, 1);
            }
            else
            {
                // If the message isn’t the proper length, just show an error:
                LCDprint("Bad data len!", 1, 1);
                LCDprint(buff, 2, 1);
            }
        }
        else
        {
            printf("No response from slave.\r\n");
            LCDprint("Signal: NONE", 2, 1);
        }
    }
    else
    {
        printf("No response from slave (timeout).\r\n");
        // Optionally, display a timeout message on the LCD.
        // LCDprint("Signal: TIMEOUT", 2, 1);
    }
    
    delayms(50);
}

// =========== PS2 CONTROLLER BIT-BANGED SPI ===========
static void ps2_delay(void)
{
    waitus(10); 
}

unsigned char PS2_TransferByte(unsigned char outByte)
{
    unsigned char inByte = 0;
    int i;
    for(i = 0; i < 8; i++)
    {
        PS2_CMD = (outByte & 0x01) ? 1 : 0;
        outByte >>= 1;
        PS2_CLK = 0;
        ps2_delay();
        if(PS2_DATA)
            inByte |= (1 << i);
        PS2_CLK = 1;
        ps2_delay();
    }
    return inByte;
}

void PS2_Init(void)
{
    ANSELBbits.ANSB0 = 0;  

    PS2_DATA_TRIS = 1;
    PS2_CMD_TRIS  = 0;
    PS2_ATT_TRIS  = 0;
    PS2_CLK_TRIS  = 0;

    PS2_CMD = 1;
    PS2_ATT = 1;
    PS2_CLK = 1;
    waitms(100);

    PS2_ATT = 0;
    PS2_TransferByte(0x01);
    PS2_TransferByte(0x43);
    PS2_TransferByte(0x00);
    PS2_TransferByte(0x01);
    PS2_TransferByte(0x00);
    PS2_ATT = 1;
    waitms(50);

    PS2_ATT = 0;
    PS2_TransferByte(0x01);
    PS2_TransferByte(0x44);
    PS2_TransferByte(0x00);
    PS2_TransferByte(0x01); // Set analog mode
    PS2_TransferByte(0x03); // Lock configuration
    PS2_ATT = 1;
    waitms(50);

    PS2_ATT = 0;
    PS2_TransferByte(0x01);
    PS2_TransferByte(0x43);
    PS2_TransferByte(0x00);
    PS2_TransferByte(0x00);
    PS2_TransferByte(0x5A);
    PS2_ATT = 1;
    waitms(50);
}

void PS2_ReadData(unsigned char *d)
{
    int i;
    PS2_ATT = 0;
    waitus(20);
    d[0] = PS2_TransferByte(0x01);
    d[1] = PS2_TransferByte(0x42);
    for(i = 2; i < 9; i++)
    {
        d[i] = PS2_TransferByte(0x00);
    }
    PS2_ATT = 1;
}

void Encode_Data(unsigned char * data, char * dest)
{
    unsigned char buttons1;
    unsigned char buttons2;
    unsigned char rx;
    unsigned char ry;
    unsigned char lx;
    unsigned char ly;

    //Encode buttons1 value
    if ((data[3]) == NONE)
    {
        dest[0] = 'N';
    }
    else if ((data[3]) == SELECT)
    {
        dest[0] = 's';
    }
    else if ((data[3]) == START)
    {
        dest[0] = 'S';
    }
    else if ((data[3]) == DPAD_UP)
    {
        dest[0] = 'U';
    }
    else if ((data[3]) == DPAD_DOWN)
    {
        dest[0] = 'D';
    }
    else if ((data[3]) == DPAD_LEFT)
    {
        dest[0] = 'L';
    }
    else if ((data[3]) == DPAD_RIGHT)
    {
        dest[0] = 'R';
    }
    else if ((data[3]) == L3)
    {
        dest[0] = 'l';
    }
    else if ((data[3]) == R3)
    {
        dest[0] = 'r';
    }
    else
    {
        dest[0] = 'E';
    }

    //Encode buttons2 value
    if ((data[4]) == NONE)
    {
        dest[1] = 'N';
    }
    else if ((data[4]) == X_BUTTON)
    {
        dest[1] = 'X';
    }
    else if ((data[4]) == SQUARE)
    {
        dest[1] = 'S';
    }
    else if ((data[4]) == O_BUTTON)
    {
        dest[1] = 'O';
    }
    else if ((data[4]) == TRIANGLE)
    {
        dest[1] = 'T';
    }
    else if ((data[4]) == R1)
    {
        dest[1] = 'r';
    }
    else if ((data[4]) == R2)
    {
        dest[1] = 'R';
    }
    else if ((data[4]) == L1)
    {
        dest[1] = 'l';
    }
    else if ((data[4]) == L2)
    {
        dest[1] = 'L';
    }
    else
    {
        dest[1] = 'E';
    }

    //Encode rx value
    if ((data[5]) >= LEVEL0 && (data[5]) < LEVEL1)
    {
        dest[2] = '1';
    }
    else if ((data[5]) >= LEVEL1 && (data[5]) < LEVEL2)
    {
        dest[2] = '2';
    }
    else if ((data[5]) >= LEVEL2 && (data[5]) < LEVEL3)
    {
        dest[2] = '3';
    }
    else if ((data[5]) >= LEVEL3 && (data[5]) < LEVEL4)
    {
        dest[2] = '4';
    }
    else if ((data[5]) >= LEVEL4 && (data[5]) < LEVEL5)
    {
        dest[2] = '5';
    }
    else if ((data[5]) >= LEVEL5 && (data[5]) < LEVEL6)
    {
        dest[2] = '6';
    }
    else if ((data[5]) >= LEVEL6 && (data[5]) < LEVEL7)
    {
        dest[2] = '7';
    }
    else if ((data[5]) >= LEVEL7 && (data[5]) < LEVEL8)
    {
        dest[2] = '8';
    }
    else if ((data[5]) >= LEVEL8 && (data[5]) < LEVEL9)
    {
        dest[2] = '9';
    }
    else
    {
        dest[2] = 'E';
    }

    //Encode ry value 
    if ((data[6]) >= LEVEL8 && (data[6]) < LEVEL9)
    {
        dest[3] = '1';
    }
    else if ((data[6]) >= LEVEL7 && (data[6]) < LEVEL8)
    {
        dest[3] = '2';
    }
    else if ((data[6]) >= LEVEL6 && (data[6]) < LEVEL7)
    {
        dest[3] = '3';
    }
    else if ((data[6]) >= LEVEL5 && (data[6]) < LEVEL6)
    {
        dest[3] = '4';
    }
    else if ((data[6]) >= LEVEL4 && (data[6]) < LEVEL5)
    {
        dest[3] = '5';
    }
    else if ((data[6]) >= LEVEL3 && (data[6]) < LEVEL4)
    {
        dest[3] = '6';
    }
    else if ((data[6]) >= LEVEL2 && (data[6]) < LEVEL3)
    {
        dest[3] = '7';
    }
    else if ((data[6]) >= LEVEL1 && (data[6]) < LEVEL2)
    {
        dest[3] = '8';
    }
    else if ((data[6]) >= LEVEL0 && (data[6]) < LEVEL1)
    {
        dest[3] = '9';
    }
    else
    {
        dest[3] = 'E';
    }    

    //Encode lx value    
    if ((data[7]) >= LEVEL0 && (data[7]) < LEVEL1)
    {
        dest[4] = '1';
    }
    else if ((data[7]) >= LEVEL1 && (data[7]) < LEVEL2)
    {
        dest[4] = '2';
    }
    else if ((data[7]) >= LEVEL2 && (data[7]) < LEVEL3)
    {
        dest[4] = '3';
    }
    else if ((data[7]) >= LEVEL3 && (data[7]) < LEVEL4)
    {
        dest[4] = '4';
    }
    else if ((data[7]) >= LEVEL4 && (data[7]) < LEVEL5)
    {
        dest[4] = '5';
    }
    else if ((data[7]) >= LEVEL5 && (data[7]) < LEVEL6)
    {
        dest[4] = '6';
    }
    else if ((data[7]) >= LEVEL6 && (data[7]) < LEVEL7)
    {
        dest[4] = '7';
    }
    else if ((data[7]) >= LEVEL7 && (data[7]) < LEVEL8)
    {
        dest[4] = '8';
    }
    else if ((data[7]) >= LEVEL8 && (data[7]) < LEVEL9)
    {
        dest[4] = '9';
    }
    else
    {
        dest[4] = 'E';
    }

    //Encode ly value 
    if ((data[8]) >= LEVEL8 && (data[8]) < LEVEL9)
    {
        dest[5] = '1';
    }
    else if ((data[8]) >= LEVEL7 && (data[8]) < LEVEL8)
    {
        dest[5] = '2';
    }
    else if ((data[8]) >= LEVEL6 && (data[8]) < LEVEL7)
    {
        dest[5] = '3';
    }
    else if ((data[8]) >= LEVEL5 && (data[8]) < LEVEL6)
    {
        dest[5] = '4';
    }
    else if ((data[8]) >= LEVEL4 && (data[8]) < LEVEL5)
    {
        dest[5] = '5';
    }
    else if ((data[8]) >= LEVEL3 && (data[8]) < LEVEL4)
    {
        dest[5] = '6';
    }
    else if ((data[8]) >= LEVEL2 && (data[8]) < LEVEL3)
    {
        dest[5] = '7';
    }
    else if ((data[8]) >= LEVEL1 && (data[8]) < LEVEL2)
    {
        dest[5] = '8';
    }
    else if ((data[8]) >= LEVEL0 && (data[8]) < LEVEL1)
    {
        dest[5] = '9';
    }
    else
    {
        dest[5] = 'E';
    }    

    //Set last char to terminate data
    dest[6] = '\n';

}

// =========== MAIN PROGRAM ===========
int main(void) {

    unsigned char ps2Data[9];
    char buff[80];
    int timeout_cnt = 0;

    // === 1) Basic PIC32 setup ===
    DDPCON = 0;
    CFGCON = 0;

	TRISBbits.TRISB12 = 0;
	LATBbits.LATB12 = 0;	
	INTCONbits.MVEC = 1;
	
	SetupTimer1(); // Setup timer 1 and its interrupt
    // Configure UART2 for debugging at 115200 baud
    UART2Configure(115200);
	
    Second_Flag = 0;
    while(!Second_Flag);

	PR2 = 0xffff; // When TMR2 hits 0xffff resets back to zero
	T2CONbits.TCKPS = 0; // Pre-scaler: 1.
	T2CONbits.TCS = 1; // External clock
	
    // Configure UART1 for JDY40 communications at 9600 baud
    UART1Configure(9600);

    delayms(500); // Allow time for terminal startup

    printf("JDY40 PS2 Master Demo. PIC32 as Master.\r\n");

    // === 2) Configure JDY40 SET pin (RB14) ===
    ANSELB &= ~(1 << 14);   // Set RB14 as digital
    TRISB  &= ~(1 << 14);   // Configure RB14 as output
    LATB   |=  (1 << 14);   // 'SET' = 1 --> normal operation mode

	ANSELB &= ~(1<<6); // Set RB3 as a digital I/O
    TRISB |= (1<<6);   // configure pin RB6 as input
    CNPUB |= (1<<6);   // Enable pull-up resistor for RB6
	T2CKRbits.T2CKR = 1; // Use RPB6 as input clock


    ReceptionOff();	
    // === 3) Send AT commands to verify/configure JDY40 (optional) ===
    SendATCommand("AT+VER\r\n");
    SendATCommand("AT+BAUD\r\n");
    SendATCommand("AT+RFID\r\n");
    SendATCommand("AT+DVID\r\n");
    SendATCommand("AT+RFC\r\n");
    SendATCommand("AT+POWE\r\n");
    SendATCommand("AT+CLSS\r\n");
	
    // Set an example unique device ID (0xABBA)
    SendATCommand("AT+RFC013\r\n");
    SendATCommand("AT+DVID0A13\r\n"); 
	
    // === 4) Initialize LCD and PS2 controller ===
    LCD_4BIT();
    waitms(50);
    LCDprint("PS2 Remote Demo", 1, 1);
    LCDprint("Initializing...", 2, 1);
    printf("Initializing PS2 and LCD...\r\n");

    PS2_Init();
    waitms(100);
    LCDprint("PS2 Ready!", 1, 1);
    LCDprint("Reading data...", 2, 1);

    // Configure the speaker pin (RB12) as output
    SPEAKER_TRIS = 0;  // Set RB12 as output
    SPEAKER = 0;       // Initialize the speaker output low

    // === 5) Main loop: Read PS2, update LCD, send data via JDY40 ===
    while(1) {
	//TMR2 = 0; // Reset timer count
	//T2_overflow=0; // Reset overflow count
	//T2CONbits.ON = 1; // Start timer  
	//Second_Flag=0;
	// Check for overflow of timer 2
	//if(IFS0&_IFS0_T2IF_MASK)
	//{
	//	IFS0CLR=_IFS0_T2IF_MASK; // Clear overflow flag
	//	T2_overflow++; // Increment overflow counter
	//}
	//T2CONbits.ON = 0; // Stop Timer
        // Call the speaker beep function based on frequency
        //beepSpeaker();	    
        // 5a) Read the PS2 data packet (9 bytes)
        PS2_ReadData(ps2Data);

        // 5e) Construct the PS2 data string to send to the slave JDY module.
        Encode_Data(ps2Data, buff);

        // 5f) Transmit the PS2 data over UART1 (JDY40)
        putc1('!');       // Send the attention character first
        delayms(5);       // Short delay
        SerialTransmit1(buff);

	delayms(20);

        // Send the request character '@' to the slave
        putc1('@');
        delayms(5);
        // Request data from the slave (this updates the LCD and speaker)
        requestSlaveData();

        // 5h) Small delay to set the pace of communication
        waitms(5);

    }

    return 0;
}
