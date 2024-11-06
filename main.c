/*
 * ProjectEM.c
 *
 * Created: 2024-11-05 2:50:00 PM
 * Author : mech458
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"
 
// define the global variables that can be used in every function  ==========
#define BRAKE 0x0F
#define CW 0x0E //backward direction of belt
#define CCW 0x0D //forward direction of belt

volatile unsigned char ADC_result;
volatile unsigned int ADC_result_flag;
volatile unsigned int killflag =0;
volatile unsigned int currBucket = 0;

int step1 = 0b00110000;
int step2 = 0b00000110;
int step3 = 0b00101000;
int step4 = 0b00000101;

void PWM();
void mTimer(int count); /* included from previous labs */
void turn(int numSteps, int dir);
void bucket(int nextBucket);
void main(int argc,char*argv[])
{
	CLKPR = 0x80; //Clock pre-scale change enable
	CLKPR = 0x01; //Required to set CPU Clock to 8MHz
	TCCR1B |= _BV(CS11); //_BV sets the bit to logic 1
	//Note the register is TCCR1B1
	//TCCR1 is the Timer/counter control register 1
	//B is the 'B' register and 1 is bit 1
	//CS means clock select, has the pre-scaler set to 8

	//Initialize LCD module
	InitLCD(LS_BLINK|LS_ULINE);
	//Clear the screen
	LCDClear();

	//Start PWM signal
	PWM();
 
	cli(); // disable all of the interrupt ==========================
	DDRL = 0xFF; /* All pins on L are set to output*/
	DDRB = 0xFF; /* All pins on B are set to output*/
	DDRC = 0xFF; /* All pins on C are set to output*/
	DDRA = 0xFF; /* All pins on A are set to output*/
	DDRD = 0x00; /* All pins on D are set to input*/
	
	PORTB = BRAKE; // init to brake

	// config the external interrupt on PD2 and PD3 ======================================
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt
	EIMSK |= (_BV(INT2)); // enable INT2
	
	EICRA |= _BV(ISC31); // falling edge interrupt
	EIMSK |= (_BV(INT3)); //enable INT3

	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN);// enableADC
	ADCSRA |= _BV(ADIE);// enable interrupt of ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0); // Result is stored in left-adjusted register (ADLAR = 1) and
	//select voltage reference selection 01 (REFS0 = 1): AVCC (analog voltage)
	//with external capacitor at AREF pin (reference pin for ADC : PB7)
	
	// sets the Global Enable for all interrupts ==========================
	sei(); 
	// initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= _BV(ADSC);

	while (1)
	{
		/* motor pseudo code
		PORTB: EA= PB3 EB = PL2(43) IA= PB1(44) IB = PB0(45)
		EA and EB = 1 always
		*/
	
		PORTA = 0b11000000;
		mTimer(1);
		PORTA = 0b00000000;
		mTimer(1);
		
		bucket(3);
		bucket(0);
		bucket(1);
		bucket(2);
		bucket(0);
		bucket(3);
		bucket(0);

		if (killflag=1)
		{
			cli();
			LCDClear()
			LCDWriteStringXY(0,1,"PRGRM KILL");
			PORTB = BRAKE; // Set all pins to Hi - brake to Vcc
		}else if (ADC_result_flag) {
			OCR0A = ADC_result; // set output compare register to ADC value
			ADC_result_flag = 0; // clear flag
			
			LCDClear();
			LCDWriteInt(ADC_result, 3); //print adc result to LCD
			mTimer(150);
			ADCSRA |= _BV(ADSC); // start another conversion
		}
	}
	return;
} //end main

//kill button

ISR(INT3_vect) {

	mTimer(20);
	
	
	killflag=1; // disable all interrupts
	
	while((PIND&0x08) == 0x00){}
	mTimer(20); //Debounce*/

}//end ISR3*/

//sensor switch: Active HIGH starts AD converstion =======
ISR(INT2_vect) {
	// when there is a rising edge, we need to do ADC =====================
	//ADCSRA |= _BV(ADSC); //adc gets started and then adc_vect will be called on completion

	//LCDWriteString("int2"); //check if isr is triggering on button press

	mTimer(20); //Debounce*/
	
	if((PIND&0x04) == 0x00){
		if (PORTB == CW || PORTB == BRAKE) {
			//brake
			PORTB = BRAKE;
			//wait 5ms for motor slow down
			mTimer(5);
			//switch to reverse motoring state
			PORTB = CCW;
		}
		else if(PORTB == CCW) {

			//brake
			PORTB = BRAKE;
			//wait 5ms for motor slow down
			mTimer(5);
			//switch to reverse motoring state
			PORTB = CW;
		}
		while((PIND&0x04) == 0x04){}
		mTimer(20); //Debounce*/
	}
}

// the interrupt will be triggered if the ADC is done ========================
ISR(ADC_vect) {
	ADC_result = ADCH;//use ADCH when using 8 bit ADC
	ADC_result_flag = 1;
}
void turn(int numSteps, int dir)
{
	//dir = 0 is ccw
	//dir = 1 is cw
	if (dir == 1) {
		for (int i = 0; i < numSteps; i++) {
			if (PORTA == step1) {
				PORTA= step2;
				mTimer(20);
			}
			else if (PORTA == step2) {
				PORTA= step3;
				mTimer(20);
			}
			else if (PORTA == step3) {
				PORTA= step4;
				mTimer(20);
			}
			else {
				PORTA= step1;
				mTimer(20);
			}
		}
	}
	else if (dir == 0) {
		for (int i = 0; i < numSteps; i++) {
			if (PORTA == step1) {
				PORTA= step4;
				mTimer(20);
			}
			else if (PORTA == step2) {
				PORTA= step1;
				mTimer(20);
			}
			else if (PORTA == step3) {
				PORTA= step2;
				mTimer(20);
			}
			else {
				PORTA= step3;
				mTimer(20);
			}
		}
	}
	mTimer(2000);
}

void mTimer (int count) {
/* The system clock is 8MHz. You can actually see the crystal oscillator(16MHz) which is the silver looking can on the board.
You can use a pre-scaler on system clock to lower the speed. The timer runs on the CPU Clock which is a function of
the system clock. You can also use a pres-scaler on the TImer, by 1, 8, 64, 256, or 1024 to lower the speed.
The system clock has been pre-scaled by 2. This means it's running at half speed, 8MHz. See the Technical manual for
ATmega2560 (ie. full manual) and look up "16-bit Timer/Counter1".*/

//Variable declarations

int i; //Keeps track of loop number

i = 0; //Initializes loop counter

//Set the Waveform Generation mode bit description to Clear Timer on  Compare Math mode (CTC) only

TCCR1B |= _BV(WGM12); //Set WGM bits to 0100, see page 145
//note WGM is spread over two registers

OCR1A = 0x03E8; /* Set output compare register for 1000 cycles= 1ms */

TCNT1 = 0x0000; /* Initialize Timer1 to zero */

//TIMSK1 = TIMSK1 | 0b00000010;  /* Enable the output compare interrupt */

TIFR1 |= _BV(OCF1A); /* Clear the Timer1 interrupt flag and begin timing */

/* Poll the timer to determine when the timer has reached 0x03E8 (1ms) */
while (i < count) {
	if ((TIFR1 & 0x02) == 0x02){

		TIFR1 |= _BV(OCF1A); /* Clear the interrupt flag by WRITING a ONE to the bit */

		i++; //Increment loop number

	} //End if

} //End while

return;
}  /* end of mTimer function */

void PWM () {
	//Set Timer 0 to Fast PWM mode - this activates mode 3 - fast PWM
	TCCR0A |= _BV(WGM00);//set to 1
	TCCR0A |= _BV(WGM01);//set to 1
	TCCR0B &= ~_BV(WGM02);//set WGM02 to 0 in TCCR0B

	//TIMSK0 |= _BV(OCIE0A);//set to 1 - enable output compare

	TCCR0A |= _BV(COM0A1);//set to 1 - set compare match output mode to clear at bottom - non-inverting mode

	//sets prescaler to clk/64
	TCCR0B |= _BV(CS00);//set to 1
	TCCR0B |= _BV(CS01);//set to 1

	OCR0A = 0x40; //scale clock to 25% duty cycle

	DDRB |= _BV(PB7); //send PWM signal to PB7
}
void bucket(int nextBucket){
//steper motor switcher based on Lab 4a code bucket: 0 = blk, 1= steel, 2 = white, 3 = aluminum
if (currBucket==0)
{
	if (nextBucket==1)
	{
		turn(50,1)//turn 90 degrees cw
		currBucket=nextBucket;
	}else if (nextBucket==3)
	{
		turn(50,0)//turn 90 degrees ccw
		currBucket=nextBucket;
	}else if (nextBucket==2)
	{
		turn(100,1)//turn 180 degrees cw
		currBucket=nextBucket;
	}
	
}
if (currBucket==1)
{
	if (nextBucket==2)
	{
		turn(50,1) //turn 90 degrees cw
		currBucket=nextBucket;
	}else if (nextBucket==0)
	{
		turn(50,0) //turn 90 degrees ccw
		currBucket=nextBucket;
	}else if (nextBucket==3)
	{
		turn(100,1) //turn 180 degrees cw
		currBucket=nextBucket;
	}
	
}
if (currBucket==2)
{
	if (nextBucket==3)
	{
		turn(50,1)//turn 90 degrees cw
		currBucket=nextBucket;
	}else if (nextBucket==1)
	{
		turn(50,0)//turn 90 degrees ccw
		currBucket=nextBucket;
	}else if (nextBucket==0)
	{
		turn(100,1)//turn 180 degrees cw
		currBucket=nextBucket;
	}
	
}
if (currBucket==3)
{
	if (nextBucket==0)
	{
		turn(50,1)//turn 90 degrees cw
		currBucket=nextBucket;
	}else if (nextBucket==2)
	{
		turn(50,0)//turn 90 degrees ccw
		currBucket=nextBucket;
	}else if (nextBucket==1)
	{
		turn(100,1)//turn 180 degrees cw
		currBucket=nextBucket;
	}
	
}
	}
