			 
#include <ADuC7026.h>
#include <string.h>
#include <stdio.h>
unsigned int PWM_PERIOD_TOP = 0xF50C; 
unsigned int PWM_duty_Time = 0x7A86;
unsigned int PWM_tempVar = 0x7A86;
unsigned int PWM_relative_duty_cycle = 5;
unsigned int INT_PERIOD_TOP = 185;
unsigned int INT_period = 0;
unsigned int INT_degree = 50;
unsigned int INT_tempVar = 50;
unsigned int INT_relative_duty_cycle = 50;
unsigned int servo_angle_speed = 1;
unsigned int A=0,B=0,C=0; 	
unsigned int Dir=1;
unsigned int adc0=0;
unsigned int distance;
unsigned int timer2=0; 
unsigned int Curve_Threshold=800;
extern int write (int file, char * ptr, int len);	// Functions used to 
extern int getchar (void);							// to output data 	 
extern int putchar(int ch);						    // Write character to Serial Port
extern int printf(const char *format, ...);

int GlobalSpeed; 															  
void Serial_Setup(void);						   // Setup functions 
void My_IRQ_Function(void);					//	IRQ Funtion Prototype  
void senddata(short) ;

void SetSpeed(int);
void UpdateSpeed(void); 
void FError(void);

void FSequence1(void);
void FSequence2(void);
void FSequence3(void);
void FSequence4(void);
void FSequence5(void);
void FSequence6(void);

void Zero_H_to_GPIO(void);
void Zero_L_to_GPIO(void);
void One_H_to_GPIO(void);
void One_L_to_GPIO(void);
void Two_H_to_GPIO(void);
void Two_L_to_GPIO(void);

void PWM_0_H(void);
void PWM_0_L(void);
void PWM_1_H(void);
void PWM_1_L(void);
void PWM_2_H(void);
void PWM_2_L(void);
void sd(int,int,int);
void RSequence1(void);
void RSequence2(void);
void RSequence3(void);
void RSequence4(void);
void RSequence5(void);
void RSequence6(void);
char hex2ascii(char) ;
void plaInitialize(void);
void sendmessage(char*) ;
void UpdateMotor();
char st=-1;
unsigned int count=0,count2=0;
void ADCpoweron(int);
void ADCinit(void);
unsigned long Test1=0x12345678,Test2;
unsigned long speed = 0x0100;
unsigned int Motor_PWM_PERIOD_TOP = 0x200;//0x0200;
unsigned int Motor_PWM_tempVar = 0x0100;
unsigned int Motor_PWM_relative_duty_cycle = 100;
int desire_speed=48;	 // Set speed .. Lower is higher ... kind of weird 
int main (void)  {
    int i=0 ;
	unsigned char jchar = 0x30;
	Serial_Setup();//setup serial code with 115200kbps 
	sendmessage("\nNatcar Group2  \n");
	sendmessage("Initialize timer...");
	T1LD = 0xA	;						// timer 1 configuration for PLA clock .. This have to be this configuration 
	T1CON = 0xC4; 
	T2LD=0;
	T2CON=0x188;
	sendmessage("Done\n");
	sendmessage("Intialize PLA ...");
	plaInitialize();
	sendmessage("Done\n");
	sendmessage("Config PWM...");
	GP3CON &= 0xFFDDDDDD; //configure P3.0 - 3.5 as PWM output
	GP3CON |= 0x00111111; 
	PWMCON = 0x01;  // 0x01 is enabled
    PWMDAT0 =  Motor_PWM_PERIOD_TOP; //0x0200; // Period register	~ 41Khz
    PWMDAT1 = 0x000; // 0 Dead time
    PWMCFG = 0x00;
	PWMEN = 0x000; //Enable all output
	ADCinit();
	sendmessage("Done\n");
   	IRQ = My_IRQ_Function;
	IRQEN = PLA_IRQ0_BIT ;
    GP1CON |= 0x03003000;
    GP3CON |= 0x03000000;
	sendmessage("Done\n")	;
	sendmessage("Version 3.0\n") ;
	while(1)
	{
	 if ((jchar-48) == 2) {
			if (Motor_PWM_relative_duty_cycle < 95) {
				Motor_PWM_relative_duty_cycle += 1;
			} else {
				Motor_PWM_relative_duty_cycle = 95;
			}
	 } else if ((jchar-48) == 8) {
	 		if (Motor_PWM_relative_duty_cycle > 55) {
				Motor_PWM_relative_duty_cycle -= 1;
			} else {
				Motor_PWM_relative_duty_cycle = 55;
			}
	 }
	 else if (jchar == 'b' || jchar=='B') //back ward
			 {
			  Dir=0;
			 }
			 else if (jchar=='f' || jchar=='F') // forward 
			 {
			 Dir=1;
			 }   
	  else  if (jchar==' ') //break
	  { 
	    Motor_PWM_relative_duty_cycle=95;
	  }

	    if (Motor_PWM_relative_duty_cycle>desire_speed && count!=count2){//((jchar-48) == 8) {
	  
	 		if (Motor_PWM_relative_duty_cycle >desire_speed) {
				Motor_PWM_relative_duty_cycle -= 1;
			} else {
				Motor_PWM_relative_duty_cycle = desire_speed ;
			}
		

			//sd(count);


	 }
	   //if (distance >= 13211) Motor_PWM_relative_duty_cycle=95;	   //  complete track then stop 
		 if ((adc0 >= 2048 + Curve_Threshold) && (adc0<=2048-Curve_Threshold))
		     Motor_PWM_relative_duty_cycle=desire_speed;
		 else
		     Motor_PWM_relative_duty_cycle=41; 
		  
		Motor_PWM_tempVar = Motor_PWM_relative_duty_cycle;				 
	 if (Motor_PWM_tempVar >= 50) {
		Motor_PWM_tempVar -= 50;					   
	 } else if (Motor_PWM_tempVar < 50) {				 
		Motor_PWM_tempVar +=50;
	 }

	speed = Motor_PWM_PERIOD_TOP * Motor_PWM_tempVar/100;

	if (Motor_PWM_relative_duty_cycle < 50) {
		speed = speed | 0x0000FF00;
	} 
 
	 
	SetSpeed(speed);
    UpdateSpeed();
	
	 while (!ADCSTA){}

	 adc0=ADCDAT>>16;
	if (!(T2VAL%18))
	 	sd(adc0,distance,timer2);


	}
}						 

void Serial_Setup(void){
	  GP1CON = 0x011;
   	// Start setting up UART at 115200bps
   	COMCON0 = 0x080;				// Setting DLAB
   	COMDIV0 = 0x0B;//0x088;				// Setting DIV0 and DIV1 to DL calculated
   	COMDIV1 = 0x00;//0x00;//0x000;
   	COMCON0 = 0x007;				// Clearing DLAB
}

void My_IRQ_Function() {				// Interupt service Routine

 if ((IRQSTA & PLA_IRQ0_BIT) !=0) 	    // Interupt from Hall Sensor
	//Figure out what the sequence is the hall sensor is at
	{	
	if (count <1000)
	    count++;
		distance++;
	   UpdateMotor();	

	}
	return ;
	}

//set the speed we want
void SetSpeed(int Speed) {
	GlobalSpeed = Speed;
}

//Change Speed
void UpdateSpeed() {
	PWMCH0 =GlobalSpeed;
	PWMCH1 =GlobalSpeed;
	PWMCH2 =GlobalSpeed;
}

//Forward Sequences. See data sheet for configuration
void FSequence1() {		  // AB	  0H =PWM  1H =X   2H = X
	UpdateSpeed();		  //      0L =X    1L =PWM 2L = X	

  /* 3.0  3.1 */
	PWM_0_H();                      
    Zero_L_to_GPIO();
  /*3.2 3.3*/
	One_H_to_GPIO();
	PWM_1_L();

  /*3.4 3.5*/
	Two_H_to_GPIO();
	Two_L_to_GPIO();
	
	
}

void FSequence2() {	       // AC	  0H =PWM  1H =X   2H = X
	UpdateSpeed();		  //          0L =X    1L =X   2L = PWM	
	/* 3.0  3.1 */
	PWM_0_H();                      
    Zero_L_to_GPIO();
	/*3.2 3.3*/
	One_H_to_GPIO();
	One_L_to_GPIO();
	/*3.4 3.5*/
	Two_H_to_GPIO();
	PWM_2_L();
}

void FSequence3() {     // BC   	  0H =X    1H =PWM   2H = X
	UpdateSpeed();		  //          0L =X    1L =X    2L = PWM	
	/* 3.0  3.1 */
	Zero_H_to_GPIO();                     
    Zero_L_to_GPIO();
   /*3.2 3.3*/
	PWM_1_H(); 
	One_L_to_GPIO();
	/*3.4 3.5*/
	Two_H_to_GPIO();
	PWM_2_L();

}

void FSequence4() {// BA  	  0H =X    1H =PWM   2H = X
	UpdateSpeed();		  //  0L =PWM    1L =X    2L = X	

	/* 3.0  3.1 */
	Zero_H_to_GPIO();                     
    PWM_0_L();
   /*3.2 3.3*/
	PWM_1_H(); 
	One_L_to_GPIO();
	/*3.4 3.5*/
	Two_H_to_GPIO();
	Two_L_to_GPIO();

}

void FSequence5() {// CA  	  0H =X     1H =X  2H = PWM 
	UpdateSpeed();		  //  0L =PWM    1L =X    2L = X	

	/* 3.0  3.1 */
	Zero_H_to_GPIO();                     
    PWM_0_L();
   /*3.2 3.3*/
	One_H_to_GPIO(); 
	One_L_to_GPIO();
	/*3.4 3.5*/
	PWM_2_H();
	Two_L_to_GPIO();

}
void FSequence6() {// CB  	  0H =X     1H =X  2H = PWM 
	UpdateSpeed();		  //  0L =X    1L =PWM   2L = X	

	/* 3.0  3.1 */
	Zero_H_to_GPIO();                     
    Zero_L_to_GPIO();
   /*3.2 3.3*/
	One_H_to_GPIO(); 
	PWM_1_L();
	
	/*3.4 3.5*/
	PWM_2_H();
	Two_L_to_GPIO();

}

//If illegal state, slow the motor down
void FError() {
	Zero_H_to_GPIO();
	One_H_to_GPIO();
	Two_H_to_GPIO();
	Zero_L_to_GPIO();
	One_L_to_GPIO();
	Two_L_to_GPIO();
}

//functions to change PWM output pins to GPIO high or low. 
void Zero_H_to_GPIO() {	      //pull LOW
	GP3CON &= 0xFFFFFFFC;	 //3.0 is High side . It's supposed to be pull HIGH in order to turn it off
	GP3DAT |= 0x01000000;
	GP3DAT &= ~(0x10000);
}

void Zero_L_to_GPIO() {		 //Pull LOW
	GP3CON &= 0xFFFFFFCF;   //3.1 is Low side . It's supposed to be push LOW in order to turn it off
	GP3DAT |= 0x02000000;
    GP3DAT &= ~(0x00020000);
}

void One_H_to_GPIO() {
	GP3CON &= 0xFFFFFCFF;
	GP3DAT |= 0x04000000;
	GP3DAT &= 0xFFFBFFFF;
	//GP3CON &= 0xFFFFFCFF;
	//GP3DAT |= 0x04000000;
	//GP3DAT |= ~(0xFFFBFFFF);
}

void One_L_to_GPIO() {
	GP3CON &= 0xFFFFCFFF;
	GP3DAT |= 0x08000000;
	GP3DAT &= ~(0x00080000);
}

void Two_H_to_GPIO() {
	GP3CON &= 0xFFFCFFFF;
	GP3DAT |= 0x10000000;
	GP3DAT &= 0xFFEFFFFF;
	//GP3CON &= 0xFFFCFFFF;
	//GP3DAT |= 0x10000000;
	//GP3DAT |= ~(0xFFEFFFFF);
}

void Two_L_to_GPIO() {
	GP3CON &= 0xFFCFFFFF;
	GP3DAT |= 0x20000000;
	GP3DAT &= ~(0x00200000);
}

//function to change GPIO pins to PWM. 
void PWM_0_H() {
	GP3CON &= 0xFFFFFFFD;
	GP3CON |= 0x00000001;
	PWMEN = 0x100;
}

void PWM_0_L() {
//	GP3CON &= 0xFFFFFFDF;
//	GP3CON |= 0x00000010;

    GP3CON &= 0xFFFFFFCF;	 //3.0 is High side . It's supposed to be pull HIGH in order to turn it off
	GP3DAT |= 0x02000000;
	GP3DAT |= 0x20000;

}

void PWM_1_H() {
	GP3CON &= 0xFFFFFDFF;
	GP3CON |= 0x00000100;
	PWMEN = 0x080;

}

void PWM_1_L() {
	//GP3CON &= 0xFFFFDFFF;
	//GP3CON |= 0x00001000; PWM on 3.2

	GP3CON &= 0xFFFFCFFF;	 //3.3 to GPIO  
	GP3DAT |= 0x08000000;	  // 3.3 to Output
	GP3DAT |= ~(0xFFF7FFFF);  // 3.3 to high 
}

void PWM_2_H() {
	GP3CON &= 0xFFFDFFFF;
	GP3CON |= 0x00010000;
	PWMEN = 0x040;
}

void PWM_2_L() {
	//GP3CON &= 0xFFDFFFFF;
	//GP3CON |= 0x00100000; // PWM 
	GP3CON &= 0xFFCFFFFF;  // 3.5 to GPIO
	GP3DAT |= 0x20000000; //  3.5 output 
	GP3DAT |= ~(0xFFDFFFFF);// 3.5 output high 
}

//Reverse Sequences
void RSequence1() {
	GP2SET &= 0xFF24FFFF; //PWM5 = 1, PWM2 = 1;
}

void RSequence2() {
	GP2SET &= 0xFF05FFFF; //PWM1 = 1, PWM2 = 1;
}

void RSequence3() {
	GP2SET &= 0xFF12FFFF; //PWM1 = 1, PWM4 = 1;
}

void RSequence4() {
	GP2SET &= 0xFF18FFFF; //PWM3 = 1, PWM4 = 1;
}

void RSequence5() {
	GP2SET &= 0xFF09FFFF; //PWM3 = 1, PWM0 = 1;
}

void RSequence6() {
	GP2SET &= 0xFF21FFFF; //PWM5 = 1, PWM0 = 1;
}


void senddata(short to_send){
 //	int save_GP1CON = GP1CON;
	GP1CON |= 0x011;
	
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
//	GP1CON	= save_GP1CON; 			
	
}




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
void     plaInitialize( )
{


    // Configure Port Pins for PLA mode

    GP1CON |= 0x03003000;	  // config 1.3, 1.6 as the PLA DIN
							  
    GP3CON |= 0x03000000;	  //config 3.6 as the input for PLA


    // Configure individual elements

    PLAELM1 = 0x04CD;
    PLAELM2 = 0x074D;
    PLAELM3 = 0x0034;
    PLAELM4 = 0x0094;
    PLAELM5 = 0x0658;
    PLAELM6 = 0x0034;
    PLAELM7 = 0x025D;
    PLAELM8 = 0x00DD;
    PLAELM11 = 0x074D;
    PLAELM13 = 0x0658;
    PLAELM14 = 0x0034;



    // Clk Source configuration

    PLACLK = 0x0055;


    // IRQ output configuration

    PLAIRQ = 0x0018;

}

void sendmessage(char* string)
{
char message[128];
memset(message,'\0',128);
sprintf(message,string) ;
write(0,message,sizeof(message));
}

void ADCpoweron(int time){
	ADCCON = 0x20;	 					// power-on the ADC
	while (time >=0)	  				// wait for ADC to be fully powered on
    time--;
}

//Called to initialize the ADC
void ADCinit(void) {
	ADCpoweron(200000);
	ADCCP = 0x00;					// selects ADC0 to read as positive channel
	ADCCON = 0x4E4;					// start conversion on timer 0
									// ADC Config: fADC/2, acq. time = 2 clocks => ADC Speed = 1MSPS	
	REFCON = 0x01;					// connect internal 2.5V reference to VREF pin
	
}
  void sd(int data,int data2,int data3)
{
  char m[128];
  int l;

  memset(m,'\0',128);
  l=sprintf(m,"%d %d  %d\n" ,data,data2,data3);
  write(0,m,l);

 }
void UpdateMotor() // this function update the motor state base on the Halls state . 
{
//Change back to GPIO input to figure out what state it's in 
 	 timer2=T2VAL;
	GP1CON &= 0xFFFFCFFF; //configure 1.3, 1.6 to be GPIO  ...  Hall 1
	GP1CON &= 0xFCFFFFFF; //configure 1.6 to be GPI0  ...  Hall 2 // change to 1.6	  
	GP3CON &= 0xFCFFFFFF; //configure 3.6 to be GPIO  ...  Hall 
	GP1DAT &= 0xB7FFFFFF; //configure 1.3, 1.6 as input
	GP1DAT &= 0xBFFFFFFF; //configure 1.6 as input
	GP3DAT &= 0xBFFFFFFF; //configure 3.6 as input
		if(((GP1DAT & 0x00000008) == 0) && ((GP1DAT & 0x00000040) == 0) && ((GP3DAT & 0x00000040) != 0)) {	//001
			if(Dir == 0x1) { 
				FSequence2();
			}
			else {
				FSequence6();
			}
		}
		else if(((GP1DAT & 0x00000008) == 0) && ((GP1DAT & 0x00000040) != 0) && ((GP3DAT & 0x00000040) != 0)) {	 //011
			if(Dir == 0x1) { 
				FSequence3();
			}
			else {
				FSequence1();
			}
		}
		else if(((GP1DAT & 0x00000008) == 0) && ((GP1DAT & 0x00000040) != 0) && ((GP3DAT & 0x00000040) == 0)) { //010
			if(Dir == 0x1) { 
				FSequence4();
			}
			else {
				FSequence2();
			}
		}
		else if(((GP1DAT & 0x00000008) != 0) && ((GP1DAT & 0x00000040) != 0) && ((GP3DAT & 0x00000040) == 0)) {	//110
			if(Dir == 0x1) { 
				FSequence5();
			}
			else {
				FSequence3();
			}
		}
		else if(((GP1DAT & 0x00000008) != 0) && ((GP1DAT & 0x00000040) == 0) && ((GP3DAT & 0x00000040) == 0)) {	//100
			if(Dir == 0x1) { 
				FSequence6();
			}
			else {
				FSequence4();
			}
		}
		else if(((GP1DAT & 0x00000008) != 0) && ((GP1DAT & 0x00000040) == 0) && ((GP3DAT & 0x00000040) != 0)) {	//101
			if(Dir == 0x1) { 
				FSequence1();
			}
			else {
				FSequence5();
			}
		}
		else {
			FError();
		} 
		GP1CON |= 0x03003000; // change back to PLA input 
        GP3CON |= 0x03000000;  // change back to PLA input
}
