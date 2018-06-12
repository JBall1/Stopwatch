//-------------------------------------------------------------------------------
//
//   Description: 
//
//   This code provides a simple stop watch on a MSP430g2553 with custom display
//   attached. The device will start at 00:00. Pressing the right most button 
//   will start the timer, going up by miliseconds. Pressing the middle button at
//   any time will reset the device to 00:00. You can press the left most button
//   at any time to pause the timer. Once the timer reaches 99:90 it will stop on
//   its own and wait for an input from the user. This all is accomplished using
//   both push button interupts for the buttons respectively and using both timers
//   on the board as well as using the dadd instruction to increment the value
//   displayed
//
//   Both Timers (TimerA_0 and TimerA_1) are set up
//   TimerA_0 will be set up to update the display at the rate of one digit
//   every 2 mS.  This results in no observable flicker in the multiplexing. 
//   Slower rates such as 10 mS per digit result in observable flicker.  
//
//   TimerA_1 will be set up to generate the 100 mS interval interrupts
//   for timing for the stopwatch or clock. TimerA_1 has the higher
//   priority of the two timers, so the time update has precedence over display
//   multiplexing. This is also where DisplayValue is incremented using the dadd
//   instruction.
//
//       Date:           April 22, 2018
//       Last Revision:  1.0
//       Written by:     Joshua Ball
//       Adapted from:   DemoWithTwoTimersC_codeRevised.c - By Dr. Helm
//       Assembler/IDE:  IAR Embedded Workbench 6.5
//
//      HW I/O assignments:
//      P1.0    (output) Segment A (active low) drives display board
//      P1.1    (output) Segment B (active low) drives display board
//      P1.2    (output) Segment C (active low) drives display board
//      P1.3    (output) Segment D (active low) drives display board
//      P1.4    (output) Segment E (active low) drives display board
//      P1.5    (output) Segment F (active low) drives display board
//      P1.6    (output) Segment G (active low) drives display board
//      P1.7    (output) Segment DP (active low) drives display board
//
//      P2.0    (output) Digit 3 (active low) MSdigit (leftmost)
//      P2.1    (output) Digit 2 (active low)  
//      P2.2    (output) Digit 1 (active low)  
//      P2.3    (output) Digit 0 (active low) LSdigit (rightmost)
//      P2.4    (output) Other - (dots - colon)
//      P2.5    (input)  Pushbutton 0 (active low) (rightmost)
//      P2.6    (input)  Pushbutton 1 (active low) (leftmost)
//      P2.7    (input)  Pushbutton 2 (active low) (middle)
//
//

#include <msp430g2553.h>
#include <stdlib.h>
//----------------------------------------------------------------------
// CONSTANT DEFINITIONS

#define MAX_RED_COUNT 2  // max number of Timer0 interrupts
#define MAX_GRN_COUNT 10 // max number of Timer1 interrupts

#define MAX_TA0_COUNT 5000 // max count for Timer0
#define MAX_TA1_COUNT 8000 // max count for Timer1

//not used
#define LED1RED (0x01) //(%00000001)  //Port pin position P1.0
#define LED2GRN (0x40) //(%01000000)  //Port pin position P1.6

//not used
#define PUSHBUTTON (00001000) //(%00001000)  //Port pin position P1.3

//Push buttons
#define PB_0  0x20		  //; Port pin position P2.5  RightMost button
#define PB_1  0x80		  //%10000000 //; Port pin position P2.7  Middle button
#define PB_2  0x40 //%01000000 //; Port pin position P2.6  LeftMost button
//ports
#define SEG_PORT P1OUT
#define DIG_PORT P2OUT
#define PB_PORT P2IN

#define ONE (0x06)
#define TWO (0x5B)
#define THREE (0x4F)
#define FOUR (0x66)
#define FIVE (0x6D)
#define SIX (0x7D)
#define SEVEN (0x07)
#define EIGHT (0x7F)
#define NINE (0x67)
#define ZERO (0x3F)

#define ONE_N ~ONE
#define TWO_N ~TWO
#define THREE_N ~THREE
#define FOUR_N ~FOUR
#define FIVE_N ~FIVE
#define SIX_N ~SIX
#define SEVEN_N ~SEVEN
#define EIGHT_N ~EIGHT
#define NINE_N ~NINE
#define ZERO_N ~ZERO

#define SEG_A (0x01)  //; Port pin position P1.0
#define SEG_B (0x02)  //; Port pin position P1.1
#define SEG_C (0x04)  //; Port pin position P1.2
#define SEG_D (0x08)  //; Port pin position P1.3
#define SEG_E (0x10)  //; Port pin position P1.4
#define SEG_F (0x20)  //; Port pin position P1.5
#define SEG_G (0x40)  //; Port pin position P1.6
#define SEG_DP (0x80) //; Port pin position P1.7

#define SEG_A_N ~SEG_A   //; Port pin position P1.0
#define SEG_B_N ~SEG_B   //; Port pin position P1.1
#define SEG_C_N ~SEG_C   //; Port pin position P1.2
#define SEG_D_N ~SEG_D   //; Port pin position P1.3
#define SEG_E_N ~SEG_E   //; Port pin position P1.4
#define SEG_F_N ~SEG_F   //; Port pin position P1.5
#define SEG_G_N ~SEG_G   //; Port pin position P1.6
#define SEG_DP_N ~SEG_DP //; Port pin position P1.7

#define DIG_3 (0x01)	  //; Port pin position P2.0 (MSdigit)
#define DIG_2 (0x02)	  //; Port pin position P2.1
#define DIG_1 (0x04)	  //; Port pin position P2.2
#define DIG_0 (0x08)	  //; Port pin position P2.3(LSdigit)
#define COL_DG_COM (0x10) //; Port pin position P2.4

#define DIG_3_N ~DIG_3			 //; Port pin position P2.0 (MSdigit)
#define DIG_2_N ~DIG_2			 //; Port pin position P2.1
#define DIG_1_N ~DIG_1			 //; Port pin position P2.2
#define DIG_0_N ~DIG_0			 //; Port pin position P2.3(LSdigit)
#define COL_DG_COM_N ~COL_DG_COM //; Port pin position P2.4

#define TIMER_A1_COUNT_1 12500
#define TIMER_A0_COUNT_1 100
//----------------------------------------------------------------------
// GLOBAL VARIABLE definition
int SegPatternTable[10] = {ZERO_N,
						   ONE_N,
						   TWO_N,
						   THREE_N,
						   FOUR_N,
						   FIVE_N,
						   SIX_N,
						   SEVEN_N,
						   EIGHT_N,
						   NINE_N};

int DigitPatternTable[5] = {DIG_0_N, DIG_1_N, DIG_2_N, DIG_3_N, COL_DG_COM_N};
int timerGO = 0;
int TotalINTCount0 = 0;
int TotalINTCount1 = 0;
int DisplayValue = 0x0000;
char CurrentDigitPos = 0;
char CurrentDigitValue = 0;
char Hundred_mS = 0;
int holder = 0;
int counter = 0;
int secCounter = 0;
int thirdCounter = 0;
//----------------------------------------------------------------------
// FUNCTIONs
//----------------------------------------------------------------------

// Function prototypes
void stop(void);
void ConfigClock(void);
void Button0(void);
void Button1(void);
void Button2(void);
void DoneWithDigitWrite(void);
void WriteNextDigitToDisplay(void);
void DoneWithDigitWriteTotally(void);
void WriteDig0(void);
void WriteDig1(void);
void WriteDig2(void);
void WriteDig3(void);
void WriteDig4(void);
void setupP2(void);
void setupP1(void);
void checkVal(int x);
// MAIN PROGRAM

void main(void) {
	WDTCTL = WDTPW + WDTHOLD; // Stop WDT
        //setupP1();
	setupP2(); // sets up P2 i/o
	ConfigClock();
      
	// Set up Timers
	// TimerA0
	TA0CCR0 = TIMER_A0_COUNT_1;		 // load a count "up to"value into timer
	TACTL = TASSEL_2 + ID_3 + MC_1; // SMCLK, input div = 8, upmode
	TA0CCTL0 = CCIE;				 //  interrupt enabled for Timer0

	// TimerA1
	TA1CCR0 = TIMER_A1_COUNT_1;		 // load a count "up to"value into timer
	TA1CTL = TASSEL_2 + ID_3+ MC_1; // SMCLK, input div = 8, upmode
	TA1CCTL0 = CCIE;				 //  interrupt enabled for Timer1
        
	_BIS_SR(GIE); // enable general interrupts
        
       // infinite loop
	for(;;){}
        
		
	

} // end of main

// not used by setups Port 1 I/O
void setupP1(void) {
	P2DIR = 0x1F;
	P1OUT = PB_0 + PB_1 + PB_2;	 //  pullup for PB
	P1REN |=  (PB_0+PB_1+PB_2);	// resistor enable for PB position
	P1IE |= PB_0 + PB_1 + PB_2;  // PB interrupt enabled
	P1IES |= PB_0 + PB_1 + PB_2; // PB Hi/lo edge
	P1IFG = 0;			 // clear ALL the Int Flags on Port 1
        SEG_PORT = (0xFF);
	DIG_PORT = (0xFF);
}
//Sets up P2 for the push buttons and 4 digit display.
void setupP2(void) {
	P2DIR = 0x1F;
	P1DIR = 0xFF;
	P2REN = (PB_0+PB_1+PB_2);
	P2OUT = PB_0 + PB_1 + PB_2;
	P2SEL &= ~(PB_1 + PB_2);
	P2IE |= PB_0 + PB_1 + PB_2;
	P2IES |= PB_0 + PB_1 + PB_2;
	P2IFG = 0; // clear ALL the Int Flags on Port 2
	SEG_PORT = (0xFF);
	DIG_PORT = (0xFF);
}
// This sets up the clock rate so it runs at 1Mhz
void ConfigClock(void) {
	DCOCTL = 0;
	DCOCTL = CALDCO_1MHZ;
	BCSCTL1 = CALBC1_1MHZ;
}

// Button functions
void stop(void) {
	timerGO = 0;
}
//Checks the given value and if it is 0x9990, stops the timer.
void checkVal(int x){
    if (DisplayValue == (0x9990)) {
      stop();
	}
}
//start
void Button0(void) {
	timerGO = 1;
	P2IFG = 0;
}
//reset
void Button1(void) {
        DisplayValue = 0x0000;
	timerGO = 0;
	P2IFG = 0;
}
//stop
void Button2(void) {
	timerGO = 0;
        P2IFG = 0;
}
// end of button functions

//These write the respective digits to the display
void WriteDig0(void) {
	DIG_PORT = (0xFF);
	int temp = 0x000F;
	int R1 = DisplayValue & temp;
	SEG_PORT = SegPatternTable[R1];
	DIG_PORT = DIG_0_N;
	CurrentDigitPos++;
}
void WriteDig1(void) {
	DIG_PORT = (0xFF);
	int temp = (0x00F0);
	int R1 = DisplayValue & temp;
	R1 = R1 >> 4;
	SEG_PORT = SegPatternTable[R1];
	DIG_PORT = DIG_1_N;
	CurrentDigitPos++;
}

void WriteDig2(void) {
	DIG_PORT = (0xFF);
	int temp = (0x0F00);
	int R1 = DisplayValue & temp;
	R1 = (R1 >> 8) | (R1 << 8);
	SEG_PORT = SegPatternTable[R1];
	DIG_PORT = DIG_2_N;
	CurrentDigitPos++;
}
void WriteDig3(void) {
        int t1 =0;
	DIG_PORT = (0xFF);
	int temp = (0xF000);
	int R1 = DisplayValue& temp;
        if(R1 == 0x8000){
          t1 = SegPatternTable[8];
          SEG_PORT = t1;
        }
         else if(R1 == 0x9000){
          t1 = SegPatternTable[9];
          SEG_PORT = t1;
        }
         else if(R1 == 0xFFFF){
          t1 = SegPatternTable[9];
          SEG_PORT = t1;
        }
        
        else{
            R1 = (R1 >> 8) | (R1 << 8);
            R1 = R1 >> 4;
            SEG_PORT = SegPatternTable[R1];
        }
	DIG_PORT = DIG_3_N;
	CurrentDigitPos++;
}
void WriteDig4(void) {
	DIG_PORT = (0xFF);
	int R15 = (0xFC);
	SEG_PORT = R15;
	DIG_PORT = COL_DG_COM_N;
	CurrentDigitPos++;
}

// Functions for WriteNextDigitToDisplay
void WriteNextDigitToDisplay(void) {
	if (CurrentDigitPos == 0) {
		WriteDig0();
	} else if (CurrentDigitPos == 1) {
		WriteDig1();
	} else if (CurrentDigitPos == 2) {
		WriteDig2();
	} else if (CurrentDigitPos == 3) {
		WriteDig3();
	} else if (CurrentDigitPos == 4) {
		WriteDig4();
	} else {
		CurrentDigitPos = 0;
	}
}

// end of functions for WriteNextDigitToDisplay

//ISR's
//Timer0_A0 ISR
#pragma vector=TIMER0_A0_VECTOR   // this line tells the C compiler to put
                                  // the start address of the following ISR
                                  // into the Interupt Vector table
//This simply updates the display with the value in DisplayValue by calling
//WriteNextDigitToDisplay
__interrupt void Timer_A0_ISR (void)   // required syntax for first line of ISR
{      
	WriteNextDigitToDisplay();
        return;
}

//ISR's  
//Timer0_A1 ISR
#pragma vector=TIMER1_A0_VECTOR   // this line tells the C compiler to put
                                  // the start address of the following ISR
                                  // into the Interupt Vector table
//This checks whether the timerGO flag has a 1 in it, and if it does then it 
//Starts updating  the display rapidly at once per milisecond stopping at 99:90.
__interrupt void Timer_A1_ISR (void)   // required syntax for first line of ISR
{       
        //used for masking out digits for insured quality of output.
        int mask1 = 0xF000;
        int mask2 = 0xFF00;
        int temp = 0xFFF0;
        //if start button was pressed..
         if (timerGO == 1) {
		Hundred_mS++;
                checkVal(DisplayValue);
		DisplayValue = DisplayValue + 0x0010;
                checkVal(DisplayValue);
                counter++;
                //if we have incremented miliseconds 9 times update second
                if(counter == 10){
                  counter = 0;
                  checkVal(DisplayValue);
                  DisplayValue = DisplayValue + 0x0100;
                  DisplayValue = DisplayValue & mask2;  
                  checkVal(DisplayValue);
                  secCounter++;
                }
                 //if we have incremented seconds 9 times update next digit
                 if(secCounter == 10){
                  secCounter = 0;
                  checkVal(DisplayValue);
                  DisplayValue = DisplayValue + 0x1000;
                  DisplayValue = DisplayValue & mask1;
                  checkVal(DisplayValue);
                  
                }
                //saftey net
                else{
                checkVal(DisplayValue);
		DisplayValue = DisplayValue & temp;
                checkVal(DisplayValue);
                }
	 }
         //if button wasn't pressed we do nothing.
	
}



// Port 1 interrupt service routine
//unused
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  	if (PB_0 == P2IFG) {
		Button0();
	} 
        if (PB_1 == P2IFG) {
		Button1();
	} 
        if (PB_2 == P2IFG) {
		Button2();
	}
  P1IFG = 0;  // clear ALL the Int Flag bits on Port 1
}
// Port 2 interrupt service routine
//This checks the buttons when a button is pressed
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void) {
	if (PB_0 == P2IFG) {
		Button0();
	} 
        if (PB_1 == P2IFG) {
		Button1();
	} 
        if (PB_2 == P2IFG) {
		Button2();
	}
        
	P2IFG = 0; // clear ALL the Int Flag bits on Port 1
        
}
