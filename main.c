/*
 * ProjectEM.c
 *
 * Created: 2024-11-08 10:05:00 AM
 * Author : mech458
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd.h"
#include "LinkedQueue.h"

 
// define the global variables that can be used in every function  ==========
#define BRAKE 0x0F
#define CW 0x0E //backward direction of belt
#define CCW 0x0D //forward direction of belt

// define global variables for stepper motor
#define STEP1 0b00110110
#define STEP2 0b00101110
#define STEP3 0b00101101
#define STEP4 0b00110101
#define STEPPER_CW 1
#define STEPPER_CCW 0

//define buckets
#define BLACK_BKT 0
#define STEEL_BKT 1
#define WHITE_BKT 2
#define ALUM_BKT 3

volatile unsigned short ADC_result;
volatile unsigned short ADC_result_old;
volatile unsigned short ADC_calc;
volatile unsigned short obj_ADC_meas;
volatile unsigned int ADC_result_flag;
volatile char STATE;

volatile unsigned int pauseflag = 0;
volatile unsigned int gate_detect = 0;
volatile unsigned int stepper_flag = 0;
volatile unsigned int ramp_flag = 0;
volatile unsigned int laser_flag = 0;
volatile int intTimer_flag = 0;

volatile unsigned int countA = 0;
volatile unsigned int countS = 0;
volatile unsigned int countB = 0;
volatile unsigned int countW = 0;

volatile unsigned int currBucket = BLACK_BKT;
//volatile unsigned int stepperDir;

void PWM();
void mTimer(int count); /* included from previous labs */
void intTimer(int count); //interrupt based timer
short readADC();
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
	
	TCCR3B |= _BV(CS31);
	
	link *head; /* The ptr to the head of the queue */
	link *tail; /* The ptr to the tail of the queue */
	link *newLink; /* A ptr to a link aggregate data type (struct) */
	link *rtnLink; /* same as the above */
	element eTest; /* A variable to hold the aggregate data type known as element */
	rtnLink = NULL;
	newLink = NULL;
	
	setup(&head, &tail);

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
	DDRA = 0xFF; 
	DDRD = 0x00; /* All pins on D are set to input*/
	DDRG = 0x00;
	
	PORTB = BRAKE; // init to brake
	

	// config the external interrupts ======================================
	//RAMP DOWN SWITCH INTERRUPT PD0:
	EICRA |= (_BV(ISC01)); // falling edge interrupt
	EIMSK |= (_BV(INT0)); //enable INT0
	
	//END OF TRAVEL SENSOR PD1:
	EICRA |= _BV(ISC11); // falling edge interrupt
	EIMSK |= (_BV(INT1)); //enable INT1
	
	//OR SENSOR TRIGGER PD2:
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt
	EIMSK |= (_BV(INT2)); // enable INT2
	
	//PAUSE SWITCH INTERRUPT PD3:
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
	//ADCSRA |= _BV(ADSC);

	while((PING&0x02) == 0x02) {
		turn(1, STEPPER_CCW);
	}//After this we are homed in on black (bin 0)
	mTimer(500);
	turn(10, STEPPER_CCW);
	mTimer(500);
	PORTB = CCW;
	
	goto POLLING_STAGE;

	// POLLING STATE
	POLLING_STAGE:
	
	switch(STATE){
		case (0) :
		goto POLLING_STAGE;
		break;	//not needed but syntax is correct
		case (1) :
		goto REFLECTIVE_STAGE;
		break;
		case (2) :
		goto BUCKET_STAGE;
		break;
		case (3) :
		goto PAUSE_STAGE;
		break;
		case (4) :
		goto RAMP_STAGE;
		break;
		case (5) :
		goto END;
		default :
		goto POLLING_STAGE;
	}//switch STATE

	REFLECTIVE_STAGE:
	// Do whatever is necessary HERE
	//ADCSRA |= _BV(ADSC); //adc gets started and then adc_vect will be called on completion
	laser_flag = 0;
	while((PIND&0x04) == 0x04) {//while there is an object in front of laser
		//EIMSK &= ~(_BV(INT2)); // disable INT2
		ADCSRA |= _BV(ADSC); //adc gets started and then adc_vect will be called on completion
		ADC_calc = (ADC_result + ADC_result_old)/2;
		if (ADC_result_flag) {//got adc value
			ADC_result_flag = 0; //clear flag
			if (ADC_calc > ADC_result){
				ADC_result_old = ADC_result;
			}
		}
	}

	obj_ADC_meas = ADC_result_old;

	//LCDClear();
	LCDWriteInt(obj_ADC_meas, 4); // To test for the numbers when using real test objects
	if (obj_ADC_meas< 300) {//add Alum to queue
		initLink(&newLink);
		newLink->e.itemCode = ALUM_BKT;
		enqueue(&head, &tail, &newLink);
	} 
	else if (obj_ADC_meas<800 && obj_ADC_meas>300 ) { //add Steel to queue
		initLink(&newLink);
		newLink->e.itemCode = STEEL_BKT;
		enqueue(&head, &tail, &newLink);
	}
	else if (obj_ADC_meas>800 && obj_ADC_meas<920) { //add white to queue
		initLink(&newLink);
		newLink->e.itemCode = WHITE_BKT;
		enqueue(&head, &tail, &newLink);
	}
	else if (obj_ADC_meas>= 920) { //add black to queue
		initLink(&newLink);
		newLink->e.itemCode = BLACK_BKT;
		enqueue(&head, &tail, &newLink);
	}
	//EIMSK |= (_BV(INT2)); // re-enable INT2
	
	//Reset the state variable
	if (ramp_flag == 1) {
		STATE = 4;
	}
	else if (pauseflag == 1) {
		STATE = 3;
	}
	else if (gate_detect == 1) {
		STATE = 2;
	}
	else if (laser_flag == 1) {
		STATE = 1;
	}
	else {
		STATE = 0;
	}
	goto POLLING_STAGE;
	
	BUCKET_STAGE:
	// Do whatever is necessary HERE
	if (gate_detect == 1) {
		gate_detect = 0; //clear flag
		//conveyor belt is stopped in interrupt
		if(size(&head, &tail) > 0) { //if queue isn't empty
			dequeue(&head, &tail, &rtnLink); //remove first item in queue (save data in rtnLink)
			bucket(rtnLink->e.itemCode); //turn to correct bin - output of FIFO
		}
		 if (intTimer_flag == 0) {
        gate_detect = 0; // Reset gate detect flag
        PORTB = CCW;     // Resume motor operation
    } else {
        // Continue ADC operations
        ADCSRA |= _BV(ADSC); // Start ADC conversion
        while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
        ADC_result = ADC;  // Read ADC value
    }
		//PORTB = CCW;//continue - this will drop item into bin
		
		//Update counts:
		if(rtnLink->e.itemCode == BLACK_BKT) {
			countB++;
		}
		else if (rtnLink->e.itemCode == STEEL_BKT) {
			countS++;
		}
		else if (rtnLink->e.itemCode == WHITE_BKT) {
			countW++;
		}
		else if(rtnLink->e.itemCode == ALUM_BKT){
			countA++;
		}
		free(rtnLink); //free memory
		
		if(size(&head, &tail) > 0) { //if queue isn't empty
			bucket(firstValue(&head).itemCode); //turn to next item bin
		}
	}

	//Reset the state variable
	if (ramp_flag == 1) {
		STATE = 4;
	}
	else if (pauseflag == 1) {
		STATE = 3;
	}
	else if (gate_detect == 1) {
		STATE = 2;
	}
	else if (laser_flag == 1) {
		STATE = 1;
	}
	else {
		STATE = 0;
	}
	goto POLLING_STAGE;
	
	PAUSE_STAGE:
		PORTB = BRAKE; // Set all pins to Hi - brake to Vcc
		LCDClear();
		LCDWriteString("PS:");
		LCDWriteInt((size(&head,&tail)), 1);
		LCDWriteString(" B:");
		LCDWriteInt(countB, 2);
		LCDWriteStringXY(0,1, "A:");
		LCDWriteIntXY(2,1,countA, 2);
		LCDWriteStringXY(5,1, "S:");
		LCDWriteIntXY(7,1, countS, 2);
		LCDWriteStringXY(10,1, "W:");
		LCDWriteIntXY(12,1, countW, 2);
		while(STATE == 3) {}
		
	//Reset the state variable
	//STATE = 0;
	goto POLLING_STAGE;
	
	RAMP_STAGE:
	if (size(&head, &tail) == 0) {
		PORTB = CCW; //Drop the last piece
		mTimer(200);
		PORTB = BRAKE; //Then stop
		goto END;
	}
	else {
		STATE = 0;
		goto POLLING_STAGE;
	}
	
	END:
	cli(); //kill interrupts
	PORTB = BRAKE; //kill the DC motor just in case
	LCDClear();
	LCDWriteString("PS:");
	LCDWriteInt((size(&head,&tail)), 1);
	LCDWriteString(" B:");
	LCDWriteInt(countB, 2);
	LCDWriteStringXY(0,1, "A:");
	LCDWriteIntXY(2,1,countA, 2);
	LCDWriteStringXY(5,1, "S:");
	LCDWriteIntXY(7,1, countS, 2);
	LCDWriteStringXY(10,1, "W:");
	LCDWriteIntXY(12,1, countW, 2);
	/*LCDWriteString("B:");
	LCDWriteInt(countB, 2);
	LCDWriteString(" A:");
	LCDWriteInt(countA, 2);
	LCDWriteStringXY(0,1, "S:");
	LCDWriteIntXY(2,1, countS, 2);
	LCDWriteStringXY(5,1, "W:");
	LCDWriteIntXY(7,1, countW, 2);*/
	
	// Stop everything here...'MAKE SAFE'
} //end main
	
//pause button -> switch to pause button later
ISR(INT3_vect) {
	mTimer(25);
	if ((PIND&0x08) == 0x00) {
		if (STATE == 3) {//currently paused
			STATE = 0; //unpause - polling state
			pauseflag = 0;
			LCDClear();
			PORTB = CCW; //continue sorting
		}
		else { //not currently paused
			PORTB = BRAKE;
			STATE=3; //pause state
			pauseflag = 1;
		}
		while((PIND&0x08) == 0x00){}
		mTimer(25); //Debounce
	}
}//end ISR3

//sensor switch: Active HIGH starts AD converstion =======
ISR(INT2_vect) {
	// when there is a rising edge, we need to do ADC =====================
	ADC_result_flag = 0; //clear adc flag
	ADCSRA |= _BV(ADSC); //start conversion
	ADC_result_old = 1024; // set old adc value to highest value
	STATE = 1; //goto reflective stage
	laser_flag = 1;
}

ISR(INT1_vect) {
	//end of conveyor belt gate sensor
	if (pauseflag == 0){
		PORTB = BRAKE;
		gate_detect = 1;
		STATE = 2;
		 // Start a background timer (e.g., 5 ms)
		 intTimer(5);
	}
}// end ISR1

ISR(INT0_vect){
	mTimer(25);
	if (pauseflag == 0) {
		ramp_flag = 1;
		STATE = 4;
	}
	while((PIND&0x01) == 0x00){}
	mTimer(25); //Debounce
}

// the interrupt will be triggered if the ADC is done ========================
ISR(ADC_vect) {
	ADC_result = ADC;
	ADC_result_flag = 1;
}

ISR(TIMER3_COMPA_vect) {
	if (intTimer_flag > 0) {
		intTimer_flag--; // Decrement the counter
		} else {
		TIMSK3 &= ~(1 << OCIE3A); // Disable interrupt when timer completes
	}
}

ISR(BADISR_vect){
 PORTL = 0xB0;
 mTimer(1000);
 PORTL = 0xA0;
 mTimer(1000);
 PORTL = 0xD0;
 mTimer(1000);
 
 STATE = 0;
}

void bucket(int nextBucket){
	if (currBucket == nextBucket) {
		return;
	}
	
	stepper_flag = 1;
	
	int step_dif = (currBucket-nextBucket + 4) % 4;
	
	if (step_dif==2) { //180 degree turn
		turn(50 * step_dif,STEPPER_CW);//turn 180 degrees cw
	}
	else if (step_dif==3) {
		turn(50,STEPPER_CCW);//turn 90 degrees ccw
	}
	else {
		turn(50,STEPPER_CW);//turn 90 degrees cw
	}
	currBucket = nextBucket;
	stepper_flag = 0;
}//end bucket

void turn(int numSteps, int dir)
{
	int accelSteps = 5;      // Number of steps for acceleration
	int decelSteps = 5;      // Number of steps for deceleration
	int steadySteps = numSteps - (accelSteps + decelSteps);     // Number of steps at constant speed

	int minDelay = 1;                   // Minimum delay for maximum speed
	int maxDelay = 15;                  // Starting delay (for slowest speed)
	int delay = maxDelay;               // Initial delay for acceleration
	
	if (numSteps < 50){
		for (int i = 0; i < numSteps; i ++) {
			if (PORTA == STEP1) {
				PORTA = STEP4;
				mTimer(20);
			}
			else if (PORTA == STEP4) {
				PORTA = STEP3;
				mTimer(20);
			}
			else if (PORTA == STEP3) {
				PORTA = STEP2;
				mTimer(20);
			}
			else {
				PORTA = STEP1;
				mTimer(20);
			}
		}
	}
	
	else if (dir == STEPPER_CW) {
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
				delay -= delay / accelSteps;  // Accelerate by reducing delay
				
				if (delay < minDelay) {
					delay = minDelay;       // Cap at minimum delay
				}
			} else if (i >= accelSteps + steadySteps) { //start decelerating now
				delay += delay / decelSteps;  // Decelerate by increasing delay
				if (delay > maxDelay) {
					delay = maxDelay; // Cap at maximum delay
				}
				//PORTB = CCW;    
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
				delay -= delay / accelSteps;
				if (delay < minDelay) {
					delay = minDelay;
				}
			} 
			else if (i >= accelSteps + steadySteps) {
				delay += delay / decelSteps;
				if (delay > maxDelay) {
					delay = maxDelay;
				}
				//PORTB = CCW;
			}
		}
	}
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
void intTimer(int count) {
	// Set Timer3 to CTC mode (WGM32 = 1, WGM33 = 0)
	TCCR3A = 0; // Normal port operation
	TCCR3B = (1 << WGM32) | (1 << CS31); // CTC mode, Prescaler = 8

	OCR3A = 0x03E8; // Set compare match value for 1ms intervals (8MHz / 8 / 1000 - 1)
	TCNT3 = 0;   // Reset the timer counter

	TIMSK3 |= (1 << OCIE3A); // Enable Compare Match A interrupt

	intTimer_flag = count; // Set the countdown value
	sei();                 // Enable global interrupts
}
 /*void intTimer (int count) {

	int i; //Keeps track of loop number
	i = 0; //Initializes loop counter

	//Set the Waveform Generation mode bit description to Clear Timer on  Compare Math mode (CTC) only

	TCCR3B |= _BV(WGM32); //Set WGM bits to 0100, see page 145
	//note WGM is spread over two registers

	OCR3A = 0x03E8; /* Set output compare register for 1000 cycles= 1ms 

	TCNT3 = 0x0000; /* Initialize Timer0 to zero 

	//TIMSK3 = TIMSK3 | 0b00000001;  /* Enable the output compare interrupt 

	TIFR3 |= _BV(OCF3A); /* Clear the Timer1 interrupt flag and begin timing 

	/* Poll the timer to determine when the timer has reached 0x03E8 (1ms) 
	while (i < count) {
		if ((TIFR3 & 0x02) == 0x02){

			TIFR3 |= _BV(OCF3A); /* Clear the interrupt flag by WRITING a ONE to the bit 

			i++; //Increment loop number

		} //End if

	} //End while

	return;
}  /* end of intTimer function */

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

	OCR0A = 0x80; //50% duty cycle

	DDRB |= _BV(PB7); //send PWM signal to PB7
}

/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/

void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
	}/*setup*/

/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
	}/*initLink*/

/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail
*  of the queue accordingly
*  INPUT: the head and tail pointers, and a pointer to the new link that was created
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/

/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink'
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink){
	*deQueuedLink = *h; // Will set to NULL if Head points to NULL
						
	if (*h != NULL) {
		*h = (*h)->next;
		if (*h == NULL) {
			*t = NULL; // Set tail to NULL if the queue is now empty
		}
	}
	return;
}/*dequeue*/

/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/

/*************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
									
	/* Last but not least set the tail to NULL */
	*t = NULL;

	return;
}/*clearQueue*/

/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	/* ENTER YOUR CODE HERE */
	return(*h == NULL);
	}/*isEmpty*/

/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
												
		return(numElements);
}/*size*/
