#include <ADuC7020.h>
#include <string.h>
#include <stdio.h>

unsigned int PWM_PERIOD_TOP = 0xF50C; 
unsigned int PWM_duty_Time = 0xA1BB;
unsigned int INT_relative_duty_cycle = 50;
unsigned int adc0 = 0;
unsigned int adc1 = 0;
unsigned int adc2 = 0;
unsigned int adc3 = 0;
unsigned int adc = 0;
int servoFeedbackTemp0 = 0;
unsigned int feedbackControlTemp = 0;
unsigned int feedback_Error_Correction_Variable = 0;
//FEEDBACK_ERROR_CORRECTION_THRESHOLD should be longer than the average step response time
//In our case, its faster than half a second.  Our feedback control runs about 1275 a second.
//To set it at half a second, we set this variable to about half-way.
unsigned int FEEDBACK_ERROR_CORRECTION_THRESHOLD = 638;
unsigned int sendOut_Variable = 0;
//feedbackControl runs at ~1275 Hz.  Set this next equality to 1275Hz/(desired Hz) to get the Hz you want.
//Example: 	Want 15ms period ~= 67Hz.
//			1275/67 ~= 19
//			set SENDOUT_THRESHOLD = 19.
unsigned int SENDOUT_THRESHOLD = 19;
char message[100] = "ghahahg";
unsigned char jchar = 0x30;
char output1[10] = "\nOk\n";

//This table is the duty cycle table for a 600 Hz PWM signal.
//With duty cycle from 51-91.  Just do duty-cycle - 51 to get
//it's position in the table.
unsigned int SERVOLOOKUPTABLE[] = 													  
{																					 
1044																			  ,
1219																			 ,
1393																			,
1567																		   ,
1741																		  ,
1915																		 ,
2089																		,
2263																	   ,
2437																	  ,
2611																	 ,
2785																	,
2959																   ,
3133																  ,
3308																 ,
3482																,
3656															   ,
3830															  ,
4004															 ,
4178															,
4352														   ,
4526														  ,
4700														 ,
4874														,
5048													   ,
5222													  ,
5396													 ,
5571													,
5745												   ,
5919												  ,
6093												 ,
6267												,
6441											   ,
6615											  ,
6789											 ,
6963											,
7137										   ,
7311										  ,
7485										 ,
7660										,
7834									   ,
8008									  ,
8182									 ,
8356									,
8530								   ,
8704								  ,
8878								 ,
9052								,
9226							   ,
9400							  ,
9574							 ,
9748							,
9923						   ,
10097						  ,
10271						 ,
10445						,
10619					   ,
10793					  ,
10967					 ,
11141					,
11315				   ,
11489				  ,
11663				 ,
11837				,
12012			   ,
12186			  ,
12360			 ,
12534			,
12708		   ,
12882		  ,
13056		 ,
13230		,
13404	   ,
13578	  ,
13752	 ,
13926	,
14100  ,
14275 ,
14449 ,
14623,
14797 ,
14971

};

unsigned int SERVO_OFFSET = 106; //53*2
unsigned int SERVO_CENTER = 146;  //73*2
unsigned int SERVO_HARD_LEFT = 186; //93*2
unsigned int SERVO_HARD_RIGHT = 106; //53*2
//last_servo_direction == 1 means last turn was a left turn
//last_servo_direction == 0 means last turn was a right turn
unsigned int last_servo_direction = 0;
unsigned int adc0Max = 0;
unsigned int adc1Max = 0;
unsigned int adcArrayCounter = 0;
unsigned int adc0Array[] = {0,0,0,0,0,0,0,0};
unsigned int adc1Array[] = {0,0,0,0,0,0,0,0};
unsigned int adc2Array[] = {0,0,0,0,0,0,0,0};
unsigned int adc3Array[] = {0,0,0,0,0,0,0,0};

/*	This number is equal to 1/kp.
	Therefore, kp = 1/kp_inverse.
	The greater kp_inverse is, the smaller kp is.
	To make kp larger, decrease kp_inverse.
*/
signed 	int KP_INVERSE = 40;
signed 	int feedbackControl_KP_term = 0;

signed	int KP_STRAIGHT_INVERSE = 43;

/*	This number is equal to 1/kd.
	Therefore, kd = 1/kd_inverse.
	The greater kd_inverse is, the smaller kd is.
	To make kd larger, decrease kd_inverse.
*/
signed	int KD_INVERSE = 60;//45;
signed 	int KD_temp = 0;
signed	int feedbackControl_KD_term = 0;
signed	int last_Error[] = { 0, 0, 0 };

signed	int feedbackSum = 0;

signed 	int	ERROR_CORRECTION_THRESHOLD = 450;
signed 	int LINE_CROSSING_THRESHOLD = 1200;

/*	Table is used to determine relative duty cycle.
	Center right now is at 74% duty cycle.  To use this table take 74+[entry_in_table]/2.
	Therefore, an entry of -37 maps to 74-18.5 = 55.5% duty cycle.
*/
 unsigned int SERVO_OFFSET_TABLE[] = {	-40,	-39,	-38,	-37,	-36,	-35,	-34,	-33,	-32,	-31,
 										-30,	-29,	-28,	-27,	-26,	-25,	-24,	-23,	-22,	-21,
										-20,	-19,	-18,	-17,	-16,	-15,	-14,	-13,	-12,	-11,
										-10,	-9,		-8,		-7,		-6,		-5,		-4,		-3,		-2,		-1,
										0,		1,		2,		3,		4,		5,		6,		7,		8,		9,
										10,		11,		12,		13,		14,		15,		16,		17,		18,		19,
										20,		21,		22,		23,		24,		25,		26,		27,		28,		29,
										30,		31,		32,		33,		34,		35,		36,		37,		38,		39,		
										40};


signed 	 int ADC_THRESHOLD_ARRAY[] = {	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0};

signed 	 int ADC_STRAIGHT_ARRAY[] = {	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0};

signed 	 int KD_THRESHOLD_ARRAY[] = {	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
										0};
//Flags
unsigned int count=0;					                                                                                                                                                                                                                   
extern int write (int file, char * ptr, int len);	// Functions used to 
extern int getchar (void);							// to output data 	 
extern int putchar(int ch);						    // Write character to Serial Port
extern int printf(const char *format, ...);
															  
void Serial_Setup(void);						   // Setup functions 
void My_IRQ_Function(void);					//	IRQ Funtion Prototype  
void senddata(short);
void ADCpoweron(int);
char hex2ascii(char);
void ADCinit(void);
void ADCrecord(void);
void feedbackControl(void);
void sendmessage(char*);
void updateServo(int);
void PWMinit(void);
void INTinit(void);
void runOffSerial(void);
void sd();
void Lookup_Table_Setup(void);

float Servo_duty_cycle=71;
//Main function initializes everything
int main (void) {
	/*Initialization code begins here */
	Serial_Setup();
	sendmessage("\nNatcar\n Version 7.5\n");
	DAC0CON = 0x12;//Enable DAC on DAC0 (pin 4 ) ;
	INTinit();	
	PWMinit();
	Lookup_Table_Setup();
    sendmessage("Initialize..done!\n");
	
	IRQ = My_IRQ_Function;			// 	Specify Interrupt Service Rountine
	ADCinit();
	IRQEN |= GP_TIMER_BIT;		//	Enable Timer1 IRQ
	IRQEN |= RTOS_TIMER_BIT;			//  Enable ADC_BIT IRQ			 1
	while(1) {	 
 		if (count==0) {
			sd();			 
		}
	} 
}

void Lookup_Table_Setup(void) {
	for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
		ADC_THRESHOLD_ARRAY[feedbackControlTemp] = SERVO_OFFSET_TABLE[feedbackControlTemp]*KP_INVERSE;
	}

	for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
		KD_THRESHOLD_ARRAY[feedbackControlTemp] = SERVO_OFFSET_TABLE[feedbackControlTemp]*KD_INVERSE;
	}

	for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
		ADC_STRAIGHT_ARRAY[feedbackControlTemp] = SERVO_OFFSET_TABLE[feedbackControlTemp]*KP_STRAIGHT_INVERSE;
	}
}
//Sets up the Serial I/O							 
void Serial_Setup(void){
	GP1CON = 0x011;

   	// Start setting up UART at 9600bps
   	COMCON0 = 0x080;				// Setting DLAB
   	COMDIV0 =  0x0b;// 0x088;				// Setting DIV0 and DIV1 to DL calculated
   	COMDIV1 = 0x000;
   	COMCON0 = 0x007;				// Clearing DLAB
}

//Interrupt.  Calls to record the ADC, and then does feedbackControl based off of the
//ADC readings.  ServoFunction is our manual PWM function
void My_IRQ_Function() {
	if ((IRQSTA & RTOS_TIMER_BIT) == 0x4) {
	   	if (count < 3000)
	    	count++;
	   	else 
	   		count = 0;
		T0CLRI = 0;
		T0LD = 0x0ED6;
		ADCrecord();
	} 
	else if ((IRQSTA & GP_TIMER_BIT) == 0x8) {			// Time 1 IRQ?
		T1CLRI = 0;
		//T1LD = 0x12E;
		T1LD = 0x1;
		feedbackControl();
	}
}

//Allows sending of data
void senddata(short to_send){
	while(!(0x020==(COMSTA0 & 0x020))){}
		COMTX = 0x0A;						// output LF 
	while(!(0x020==(COMSTA0 & 0x020))){}
		COMTX = 0x0D;						// output CR 
	while(!(0x020==(COMSTA0 & 0x020))){}
		COMTX = hex2ascii ((to_send >> 8) & 0x0F);
	while(!(0x020==(COMSTA0 & 0x020))){}
		COMTX = hex2ascii ((to_send >> 4) & 0x0F);						
	while(!(0x020==(COMSTA0 & 0x020))){}
		COMTX = hex2ascii (to_send & 0x0F);	
}

//Turns Hex to Ascii
char hex2ascii(char toconv) {
	if (toconv<0x0A) 
	{
		toconv += 0x30;
	}
	else 
	{
		toconv += 0x37;
	}
	
	return (toconv);
}

//One of the ADC initialization methods
void ADCpoweron(int time){
	ADCCON = 0x20;	 					// power-on the ADC
	while (time >=0)	  				// wait for ADC to be fully powered on
    time--;
}

//Called to initialize the ADC
void ADCinit(void) {
	ADCpoweron(20000);
	ADCCP = 0x01;					// selects ADC0 to read as positive channel
	ADCCON = 0x4E4;					// start conversion on timer 0
									// ADC Config: fADC/2, acq. time = 2 clocks => ADC Speed = 1MSPS	
	REFCON = 0x01;					// connect internal 2.5V reference to VREF pin
	
	//timer0 configuration
	T0LD = 0x0D55;					// 3413/40.96MHz = 12000Hz
	T0CON = 0xC0;					// count down
									// periodic mode
}

//Used to record the ADC values.  Currently switches reading between ADC0 and ADC1,
void ADCrecord(void) {
	if (adc == 0) {
		adc0Array[adcArrayCounter] = ADCDAT >> 16;
		adc0 = (adc0Array[0] + adc0Array[1] + adc0Array[2] + adc0Array[3] + adc0Array[4] + adc0Array[5] + adc0Array[6] + adc0Array[7]) >> 3;//(adc0Array[1] >> 2) + (adc0Array[2] >> 1) + (adc0Array[3] >> 1) + (adc0Array[4] >> 2) - (adc0Array[0] >> 4) - (adc0Array[5] >> 4);
		adc = 1;
		ADCCP = 0x02;
	} else if (adc == 1) {
		adc1Array[adcArrayCounter] = ADCDAT >> 16;
		adc1 = (adc1Array[0] + adc1Array[1] + adc1Array[2] + adc1Array[3] + adc1Array[4] + adc1Array[5] + adc1Array[6] + adc1Array[7]) >> 3;//(adc1Array[1] >> 2) + (adc1Array[2] >> 1) + (adc1Array[3] >> 1) + (adc1Array[4] >> 2) - (adc1Array[0] >> 4) - (adc1Array[5] >> 4);
		adc = 2;
		ADCCP = 0x03;
	} else if (adc == 2) {
		DAC0DAT=ADCDAT;
		adc2Array[adcArrayCounter] = ADCDAT >> 16;
		adc2 = (adc2Array[0] + adc2Array[1] + adc2Array[2] + adc2Array[3] + adc2Array[4] + adc2Array[5] + adc2Array[6] + adc2Array[7]) >> 3;//(adc2Array[1] >> 2) + (adc2Array[2] >> 1) + (adc2Array[3] >> 1) + (adc2Array[4] >> 2) - (adc2Array[0] >> 4) - (adc2Array[5] >> 4);
		adc = 3;
		ADCCP = 0x04;
	} else if (adc == 3) {
		adc3Array[adcArrayCounter] = ADCDAT >> 16;
		adc3 = (adc3Array[0] + adc3Array[1] + adc3Array[2] + adc3Array[3] + adc3Array[4] + adc3Array[5] + adc3Array[6] + adc3Array[7]) >> 3;//(adc3Array[1] >> 2) + (adc3Array[2] >> 1) + (adc3Array[3] >> 1) + (adc3Array[4] >> 2) - (adc3Array[0] >> 4) - (adc3Array[5] >> 4);
		adc = 0;
		ADCCP = 0x01;

		if (adcArrayCounter >= 7) {
			adcArrayCounter = 0;
		} else {
			adcArrayCounter += 1;
		}
	}
	//ADCCP ^= 1;				//flip to read the other ADC - ADC0 and ADC1;
	//change PWM_relative_duty_cycle depending on the value of adc0 and adc1
	//remember that the servo's resolution is only from 35% to 65% duty cycle
}

//Feedback control method.  It first checks the error case that the car is off
//track.  Then it updates the servo based off of a linearized function.
void feedbackControl(void) {
	servoFeedbackTemp0 = adc1-adc0;

	//If line crossing, keep the same steering angle
	//Else if need to error correct, turn hard in the last turned direction
	//Else do regular line following

/*	if (adc3 > LINE_CROSSING_THRESHOLD) {
		updateServo(INT_relative_duty_cycle);

		if ((GP1DAT & 0x10) == 0x10) {
			for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
				if (feedbackControlTemp == 0) {
					if (servoFeedbackTemp0 <= ADC_STRAIGHT_ARRAY[feedbackControlTemp]) {
						feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];			
						break;
					}
				} else if ((servoFeedbackTemp0 <= ADC_THRESHOLD_ARRAY[feedbackControlTemp]) && (servoFeedbackTemp0 >= ADC_THRESHOLD_ARRAY[feedbackControlTemp - 1])) {
					feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];	
					break;
				}
			}
		} else {
			for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
				if (feedbackControlTemp == 0) {
					if (servoFeedbackTemp0 <= ADC_STRAIGHT_ARRAY[feedbackControlTemp]) {
						feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];			
						break;
					}
				} else if ((servoFeedbackTemp0 <= ADC_THRESHOLD_ARRAY[feedbackControlTemp]) && (servoFeedbackTemp0 >= ADC_THRESHOLD_ARRAY[feedbackControlTemp - 1])) {
					feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];	
					break;
				}
			}
		}

		KD_temp = (6*(last_Error[2]-servoFeedbackTemp0) + 2*(last_Error[1]-last_Error[0]))/20;

		for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
			if (KD_temp <= KD_THRESHOLD_ARRAY[feedbackControlTemp]) {
				feedbackControl_KD_term = SERVO_OFFSET_TABLE[feedbackControlTemp];
				break;
			}
		}

		feedbackSum = feedbackControl_KP_term - feedbackControl_KD_term;

		if (feedbackSum < 0) {
			last_servo_direction = 0;
		} else if (feedbackSum > 0) {
			last_servo_direction = 1;
		}

	} else
	*/ 
	if (adc2 < ERROR_CORRECTION_THRESHOLD) {
		if (last_servo_direction == 0) {
			updateServo(SERVO_HARD_RIGHT);
		} else if (last_servo_direction == 1) {
			updateServo(SERVO_HARD_LEFT);
		}
	} else {
		
		//if straight
	/*	if ((GP1DAT & 0x10) == 0x10) {
			for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
				if (feedbackControlTemp == 0) {
					if (servoFeedbackTemp0 <= ADC_STRAIGHT_ARRAY[feedbackControlTemp]) {
						feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];			
						break;
					}
				} else if ((servoFeedbackTemp0 <= ADC_THRESHOLD_ARRAY[feedbackControlTemp]) && (servoFeedbackTemp0 >= ADC_THRESHOLD_ARRAY[feedbackControlTemp - 1])) {
					feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];	
					break;
				}
			}
		} else {*/
			for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
				if (feedbackControlTemp == 0) {
					if (servoFeedbackTemp0 <= ADC_STRAIGHT_ARRAY[feedbackControlTemp]) {
						feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];			
						break;
					}
				} else if ((servoFeedbackTemp0 <= ADC_THRESHOLD_ARRAY[feedbackControlTemp]) && (servoFeedbackTemp0 >= ADC_THRESHOLD_ARRAY[feedbackControlTemp - 1])) {
					feedbackControl_KP_term = SERVO_OFFSET_TABLE[feedbackControlTemp];	
					break;
				}
			}
	//	}

		KD_temp = (6*(last_Error[2]-servoFeedbackTemp0) + 2*(last_Error[1]-last_Error[0]))/20; 
		//KD_temp = ((last_Error[2]-servoFeedbackTemp0)<<2 + (last_Error[2]-servoFeedbackTemp0+last_Error[1]-last_Error[0])<<1 )/20; 
		for (feedbackControlTemp = 0; feedbackControlTemp < 81; feedbackControlTemp++) {
			if (KD_temp <= KD_THRESHOLD_ARRAY[feedbackControlTemp]) {
				feedbackControl_KD_term = SERVO_OFFSET_TABLE[feedbackControlTemp];
				break;
			}
		}

		feedbackSum = feedbackControl_KP_term - feedbackControl_KD_term;
		updateServo(SERVO_CENTER + feedbackSum);
		if (feedbackControl_KP_term < 0) {
			last_servo_direction = 0;
		} else if (feedbackControl_KP_term > 0) {
			last_servo_direction = 1;
		}
	}

	last_Error[2] = last_Error[1];
	last_Error[1] = last_Error[0];
	last_Error[0] = servoFeedbackTemp0;
}

//Allow's sending of messages
void sendmessage(char* string){
	char message[128];
	int l;

	memset(message,'\0',128);
    l=sprintf(message,string) ;
	write(0,message,l);
}

//Update Servo duty cycle method
void updateServo(int dutyCycle) {
	INT_relative_duty_cycle = dutyCycle;
	if ((dutyCycle >= SERVO_HARD_RIGHT) && (dutyCycle <= SERVO_HARD_LEFT)) {
		PWMCH0 = SERVOLOOKUPTABLE[dutyCycle - SERVO_OFFSET];	
	} else if (dutyCycle <= SERVO_HARD_RIGHT) {
		PWMCH0 = SERVOLOOKUPTABLE[SERVO_HARD_RIGHT - SERVO_OFFSET];
	} else if (dutyCycle >= SERVO_HARD_LEFT) {
		PWMCH0 = SERVOLOOKUPTABLE[SERVO_HARD_LEFT - SERVO_OFFSET];
	} else {
		//Set the PWM to center
		PWMCH0 = 7311;
	}
}

//Initializes the PWM
void PWMinit(void) {
	PWMCON = 0x1;
	PWMDAT0 = 34816;//0x8800;  //600Hz
	PWMCH0 = 0x0;
	GP4CON = 0x300;
	GP3CON = 0x1;	 //configure 3.0
	PLAELM8 = 0x0035;
	PLAELM10 = 0x0059;
}

//Initializes the timer for the servo
void INTinit(void) {
	//T1LD = 0x12E;						// timer 1 configuration	 
	//T1CON = 0xC4;
	T1LD = 0x1;
	T1CON = 0xCF;
}

//Debugging method
 void sd()
{
  	char m[214];
 	int l;

  	memset(m,'\0',214);
  	l=sprintf(m,"%d %d %d %d %d %d %d\n", adc1-adc0, adc0, adc1, adc2, adc3, feedbackSum, last_servo_direction);// INT_relative_duty_cycle, PWMCH0); //adc1-adc0
	write(0,m,l);

 }
