			 
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

void RSequence1(void);
void RSequence2(void);
void RSequence3(void);
void RSequence4(void);
void RSequence5(void);
void RSequence6(void);
char hex2ascii(char) ;
void plaInitialize(void);
void sendmessage(char*) ;
char st=-1;
char flag;
int count=0,count2=0;
unsigned long Test1=0x12345678,Test2;
unsigned int speed = 0x0100;
unsigned int Motor_PWM_PERIOD_TOP = 0x200;//0x0200;
unsigned int Motor_PWM_tempVar = 0x0100;
unsigned int Motor_PWM_relative_duty_cycle = 100;
int main (void)  {
	unsigned char jchar = 0x30;
//	char buf[128];
	//mset(buf,'\0',128);
	//sendmessage("Hell World!!");
	/*Initialization code begins here */
	Serial_Setup();
	sendmessage("\nNatcar Group2 \n");
	sendmessage("Version 1.9\n") ;
	sendmessage("Initializing...\n");
	T0LD = 0x1;						// timer 1 configuration	 
	T0CON = 0xC8; 
	T1LD = 0xA	;						// timer 1 configuration	 
	T1CON = 0xC4; 
	GP4CON = 0x00000003	 ;
	GP4DAT = 0x05000000;				//	Configure P4.2 as output
	GP0CON = 0x03300000;  
	GP0DAT = 0xC0000000; //configure 0.4 0.5 as the input
	GP1DAT = 0x00000000; //configure 1.4 1.5 as the input;
	GP2CON = 0x00000000; // configure P2.3 as an GPIO;
	GP2DAT = 0x08000000;  // configure P2.3 as an output;
	//GP2DAT = 0xFFF7FFFF;
	sendmessage("Intialize PLA ...\n");
	plaInitialize();
	sendmessage("Config PWM...\n");
	GP3CON &= 0xFFDDDDDD; //configure P3.0 - 3.5 as PWM output
	GP3CON |= 0x00111111; 
	PWMCON = 0x01;   			// 0x01 is enabled
    PWMDAT0 =  Motor_PWM_PERIOD_TOP; //0x0200; // Period register	~ 41Khz
    PWMDAT1 = 0x000; // 0 Dead time
    PWMCFG = 0x00;
	PWMEN = 0x000; //Enable all output

   IRQ = My_IRQ_Function;
	IRQEN = PLA_IRQ0_BIT;//;+ RTOS_TIMER_BIT;	
	sendmessage("Config PWM...\n");
    
	//IRQEN = PLA_IRQ0_BIT;
	//return ;
	





	

	/*Initialization code ends here */

	
	/* Running code begins here */
	sendmessage("Done\n")	;
	while(1)
	{		
      


	 if ((jchar-48) == 2) {
			if (Motor_PWM_relative_duty_cycle < 95) {
				Motor_PWM_relative_duty_cycle += 1;
			} else {
				Motor_PWM_relative_duty_cycle = 95;
			}
	 } else if ((jchar-48) == 8) {
	 		if (Motor_PWM_relative_duty_cycle > 3) {
				Motor_PWM_relative_duty_cycle -= 1;
			} else {
				Motor_PWM_relative_duty_cycle = 3;
			}
	 }
	 else if (jchar == 'b' || jchar=='B')
			 {
			  Dir=0;
			 }
			 else if (jchar=='f' || jchar=='F')
			 {
			 Dir=1;
			 }
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
//	//sprintf(buf,"Speed= %d \n ",speed);
	  	// senddata(speed);
////	 write(0,buf,40);

// write(0,&speed,4);
senddata(Motor_PWM_tempVar>>24);
	senddata(Motor_PWM_tempVar>>12);
	senddata(Motor_PWM_tempVar);
   sendmessage("\n?>");
	jchar = getchar();
	
	}
}							 

void Serial_Setup(void){
	  GP1CON = 0x011;
   	// Start setting up UART at 9600bps
   	COMCON0 = 0x080;				// Setting DLAB
   	COMDIV0 = 0x0B;//0x088;				// Setting DIV0 and DIV1 to DL calculated
   	COMDIV1 = 0x00;//0x00;//0x000;
   	COMCON0 = 0x007;				// Clearing DLAB
}

void My_IRQ_Function() {				

 
Test1=IRQSIG;
 
 Test2=IRQSTA ;
 
 
	if ((IRQSTA & GP_TIMER_BIT) !=0) 			// Time 1 IRQ?
	{ 

	//	T1CLRI = 0;
	//	T1LD = 0x1;
		//INT_period += 1;
	
	}
		

		
	
		//Clear the Interrupt bit
		
		 

 if ((IRQSTA & PLA_IRQ0_BIT) !=0) 	    // Interupt from Hall Sensor
	//Figure out what the sequence is the hall sensor is at
		
	{	
	
	  
              	//  if (count==1) count2++;
	GP1CON &= 0xFFFFCFFF; //configure 1.3, 1.6 to be GPIO  ...  Hall 1
	GP1CON &= 0xFCFFFFFF; //configure 1.6 to be GPI0  ...  Hall 2 // change to 1.6	  
	GP3CON &= 0xFCFFFFFF; //configure 3.6 to be GPIO  ...  Hall 
	GP1DAT &= 0xB7FFFFFF; //configure 1.3, 1.6 as input
	GP1DAT &= 0xBFFFFFFF; //configure 1.6 as input
	GP3DAT &= 0xBFFFFFFF; //configure 3.6 as input
	
		if(((GP1DAT & 0x00000008) == 0) && ((GP1DAT & 0x00000040) == 0) && ((GP3DAT & 0x00000040) != 0)) {	//001
			if(Dir == 0x1) { 
				FSequence2();
				//st=2;
			//	sendmessage("\n2\n");
				
			}
			else {
				FSequence6();
			}
		}
		else if(((GP1DAT & 0x00000008) == 0) && ((GP1DAT & 0x00000040) != 0) && ((GP3DAT & 0x00000040) != 0)) {	 //011
			if(Dir == 0x1) { 
				FSequence3();
				//st=3;
				//	sendmessage("\n3\n");
			}
			else {
				FSequence5();
			}
		}
		else if(((GP1DAT & 0x00000008) == 0) && ((GP1DAT & 0x00000040) != 0) && ((GP3DAT & 0x00000040) == 0)) { //010
			if(Dir == 0x1) { 
				FSequence4();
					
				//		sendmessage("\n4\n");
			}
			else {
				FSequence4();
			}
		}
		else if(((GP1DAT & 0x00000008) != 0) && ((GP1DAT & 0x00000040) != 0) && ((GP3DAT & 0x00000040) == 0)) {	//110
			if(Dir == 0x1) { 
				FSequence5();
			
				//	sendmessage("\n5\n");
			}
			else {
				FSequence3();
			}
		}
		else if(((GP1DAT & 0x00000008) != 0) && ((GP1DAT & 0x00000040) == 0) && ((GP3DAT & 0x00000040) == 0)) {	//100
			if(Dir == 0x1) { 
				FSequence6();
				//	sendmessage("\n6\n");
			}
			else {
				FSequence2();
			}
		}
		else if(((GP1DAT & 0x00000008) != 0) && ((GP1DAT & 0x00000040) == 0) && ((GP3DAT & 0x00000040) != 0)) {	//101
			if(Dir == 0x1) { 
				FSequence1();
				//	 sendmessage("\n1\n");
			}
			else {
				FSequence1();
			}
		}
		else {
			FError();
				//sendmessage("\nError\n");
		} 
		GP1CON |= 0x03003000;

    GP3CON |= 0x03000000;
	
	
	   
	 }

	
	return ;
	}

//set the speed we want
void SetSpeed(int Speed) {
	GlobalSpeed = Speed;
}

//Change Speed
void UpdateSpeed() {
	PWMCH0 = GlobalSpeed;
	PWMCH1 = GlobalSpeed;
	PWMCH2 = GlobalSpeed;
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
void Zero_H_to_GPIO() {	      //pull HIGH
	GP3CON &= 0xFFFFFFFC;	 //3.0 is High side . It's supposed to be pull HIGH in order to turn it off
	GP3DAT |= 0x01000000;
	GP3DAT &= ~(0x10000);
	//GP3CON &= 0xFFFFFFFC;	 //3.0 is High side . It's supposed to be pull HIGH in order to turn it off
	//GP3DAT |= 0x01000000;
	//GP3DAT |= 0x10000;
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
	GP3CON &= 0xFFFFFFDF;
	GP3CON |= 0x00000010;
}

void PWM_1_H() {
	GP3CON &= 0xFFFFFDFF;
	GP3CON |= 0x00000100;
	PWMEN = 0x080;
}

void PWM_1_L() {
	GP3CON &= 0xFFFFDFFF;
	GP3CON |= 0x00001000;
}

void PWM_2_H() {
	GP3CON &= 0xFFFDFFFF;
	GP3CON |= 0x00010000;
	PWMEN = 0x040;
}

void PWM_2_L() {
	GP3CON &= 0xFFDFFFFF;
	GP3CON |= 0x00100000;
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


