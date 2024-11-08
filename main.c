/*
 * ProjectEM.c
 *
 * Created: 2024-11-08 10:05:00 AM
 * Author : mech458
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"
 
// define the global variables that can be used in every function  ==========
#define BRAKE 0x0F
#define CW 0x0E //backward direction of belt
#define CCW 0x0D //forward direction of belt

// define global variables for stepper motor
#define STEP1 0b00110000
#define STEP2 0b00000110
#define STEP3 0b00101000
#define STEP4 0b00000101
#define STEPPER_CW 1
#define STEPPER_CCW 0

//define buckets
#define BLACK_BKT 0
#define STEEL_BKT 1
#define WHITE_BKT 2
#define ALUM_BKT 3

volatile unsigned short ADC_result;
volatile unsigned short ADC_result_old;
volatile unsigned char ADCH_result;
volatile unsigned char ADCL_result;
volatile unsigned int ADC_result_flag;

volatile unsigned int killflag = 0;

volatile unsigned int gate_detect = 0;

volatile unsigned int currBucket = BLACK_BKT;
//volatile unsigned int stepperDir;

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

	// config the external interrupts ======================================
	//END OF TRAVEL SENSOR PD1:
	EICRA |= _BV(ISC11); // falling edge interrupt
	EIMSK |= (_BV(INT1)); //enable INT1
	
	//OR SENSOR TRIGGER PD2:
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt
	EIMSK |= (_BV(INT2)); // enable INT2
	
	//KILL SWITCH INTERRUPT PD3:
	EICRA |= _BV(ISC31); // falling edge interrupt
	EIMSK |= (_BV(INT3)); //enable INT3

	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN);// enableADC
	ADCSRA |= _BV(ADIE);// enable interrupt of ADC -> for RL sensor
	ADMUX |= _BV(REFS0); 
	//ADMUX |= _BV(ADLAR); // Result is stored in left-adjusted register (ADLAR = 1) and
	//select voltage reference selection 01 (REFS0 = 1): AVCC (analog voltage)
	//with external capacitor at AREF pin (reference pin for ADC : PB7)
	
	// sets the Global Enable for all interrupts ==========================
	sei(); 
	// initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= _BV(ADSC);

	while (1)
	{
		//PORTL = 0x00;
		/*PORTA = 0b11000000;
		mTimer(1);
		PORTA = 0b00000000;
		mTimer(1);
		
		mTimer(5000);
		bucket(1);
		bucket(3);
		bucket(1);
		bucket(2);
		bucket(0);
		bucket(2);
		bucket(3);
		bucket(0);*/

		if (killflag == 1) {
			cli();
			LCDClear();
			LCDWriteStringXY(0,1,"PRGRM KILL");
			PORTB = BRAKE; // Set all pins to Hi - brake to Vcc
		}
		else if (ADC_result_flag) {
			//laser has been blocked by something - value stored in ADC_result
			ADC_result_flag = 0; // clear ADC flag
			if (ADC_result < 0b1110000100) {
				PORTL = 0xF0;
			}
			LCDClear();
			//here do stuff with adc result: determine item type
			if (ADC_result_old < ADC_result) {
				//we have lowest value of ADC reading in ADC_result_old
				//classify item and input into FIFO
			}
			
			//ADCSRA |= _BV(ADSC);
			ADC_result_old = ADC_result;//store current ADC result to check with next one
		}
		else if (gate_detect) {
			gate_detect = 0; //clear flag
			//stop DC motor
			//turn to correct bin - output of FIFO
			//continue - this will drop item into bin
		}
	}
	return;
} //end main

//kill button -> switch to pause button later

ISR(INT3_vect) {

	mTimer(20);
	
	killflag=1; // disable all interrupts
	
	while((PIND&0x08) == 0x00){}
	mTimer(20); //Debounce

}//end ISR3

//sensor switch: Active HIGH starts AD converstion =======
ISR(INT2_vect) {
	// when there is a rising edge, we need to do ADC =====================
	ADCSRA |= _BV(ADSC); //adc gets started and then adc_vect will be called on completion
}

ISR(INT1_vect) {
	//end of conveyor belt gate sensor
	gate_detect = 1;
}// end ISR1

// the interrupt will be triggered if the ADC is done ========================
ISR(ADC_vect) {
	ADCL_result = ADCL; // Read ADCL first
	ADCH_result = ADCH;//Read ADCH second
	ADC_result = (ADCH_result << 8) | ADCL_result; //ADCH value in higher 8 bits,  2 useful bits in ADCL in lower 2 bits, total 10 bits
	ADC_result_flag = 1;
}
/*ISR(BADISR_vect){
 PORTC = 0xB0;
 mTimer(1000);
 PORTC = 0xA0;
 mTimer(1000);
 PORTC = 0xD0;
 mTimer(1000);
}*/
void turn(int numSteps, int dir)
{
	int accelSteps = 7;      // Number of steps for acceleration
	int decelSteps = 7;      // Number of steps for deceleration
	int steadySteps = numSteps - 14;     // Number of steps at constant speed

	int minDelay = 5;                   // Minimum delay for maximum speed
	int maxDelay = 20;                  // Starting delay (for slowest speed)
	int delay = maxDelay;               // Initial delay for acceleration

	if (dir == STEPPER_CW) {
		for (int i = 1; i < (numSteps+1); i++) { //i = number of steps taken
			// Set the port values for each step
			if (PORTA == STEP1) {
				PORTA = STEP2;
			}
			else if (PORTA == STEP2) {
				PORTA = STEP3;
			}
			else if (PORTA == STEP3) {
				PORTA = STEP4;
			}
			else {
				PORTA = STEP1;
			}

			mTimer(delay);  // Wait for the current delay

			// Adjust delay for acceleration
			if (i < accelSteps) {
				delay -= (maxDelay - minDelay) / accelSteps;  // Accelerate by reducing delay
				
				if (delay < minDelay) {
					delay = minDelay;       // Cap at minimum delay
				}
			} else if (i >= accelSteps + steadySteps) { //start decelerating now
				delay += (maxDelay - minDelay) / decelSteps;  // Decelerate by increasing delay
				
				if (delay > maxDelay) {
					delay = maxDelay; // Cap at maximum delay
				}      
			}
		}
	} 
	else if (dir == STEPPER_CCW) { // Counter-clockwise movement
		for (int i = 1; i < (numSteps); i++) {
			// Set the port values for each step in reverse
			if (PORTA == STEP1) {
				PORTA = STEP4;
			}
			else if (PORTA == STEP4) {
				PORTA = STEP3;
			}
			else if (PORTA == STEP3) {
				PORTA = STEP2;
			}
			else {
				PORTA = STEP1;
			}

			mTimer(delay);  // Wait for the current delay

			// Adjust delay for acceleration, steady, and deceleration phases
			if (i < accelSteps) {
				delay -= (maxDelay - minDelay) / accelSteps;
				if (delay < minDelay) {
					delay = minDelay;
				}
			} 
			else if (i >= accelSteps + steadySteps) {
				delay += (maxDelay - minDelay) / decelSteps;
				if (delay > maxDelay) {
					delay = maxDelay;
				}
			}
		}
	}
	mTimer(2000); // Pause briefly after each full movement
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
	//stepper motor switcher based on Lab 4a code bucket: 0 = blk, 1= steel, 2 = white, 3 = aluminum
	if (currBucket==BLACK_BKT) {
		if (nextBucket==STEEL_BKT) {
			//stepperDir = STEPPER_CCW;
			turn(50,STEPPER_CCW);//turn 90 degrees ccw
			currBucket=nextBucket;
		}
		else if (nextBucket==ALUM_BKT) {
			//stepperDir = STEPPER_CW;
			turn(50,STEPPER_CW);//turn 90 degrees cw
			currBucket=nextBucket;
		}
		else if (nextBucket==WHITE_BKT) {
			turn(100,STEPPER_CW);//turn 180 degrees cw
			currBucket=nextBucket;
		}
	}
	else if (currBucket==STEEL_BKT) {
		if (nextBucket==WHITE_BKT) {
			turn(50,STEPPER_CCW); //turn 90 degrees ccw
			currBucket=nextBucket;
		}
		else if (nextBucket==BLACK_BKT) {
			turn(50,STEPPER_CW); //turn 90 degrees cw
			currBucket=nextBucket;
		}
		else if (nextBucket==ALUM_BKT) {
			turn(100,STEPPER_CW); //turn 180 degrees cw
			currBucket=nextBucket;
		}
	}
	else if (currBucket==WHITE_BKT) {
		if (nextBucket==ALUM_BKT) {
			turn(50,STEPPER_CCW);//turn 90 degrees ccw
			currBucket=nextBucket;
		}
		else if (nextBucket==STEEL_BKT) {
			turn(50,STEPPER_CW);//turn 90 degrees cw
			currBucket=nextBucket;
		}
		else if (nextBucket==BLACK_BKT) {
			turn(100,STEPPER_CW);//turn 180 degrees cw
			currBucket=nextBucket;
		}
	}
	else if (currBucket==ALUM_BKT) {
		if (nextBucket==BLACK_BKT) {
			turn(50,STEPPER_CCW);//turn 90 degrees ccw
			currBucket=nextBucket;
		}
		else if (nextBucket==WHITE_BKT) {
			turn(50,STEPPER_CW);//turn 90 degrees cw
			currBucket=nextBucket;
		}
		else if (nextBucket==STEEL_BKT) {
			turn(100,STEPPER_CW);//turn 180 degrees cw
			currBucket=nextBucket;
		}
	}
}//end bucket
