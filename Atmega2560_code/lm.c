#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#define F_CPU 14745600
unsigned char data=0x00; //to store received data from UDR1
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
int orientation=0;
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	buzzer_pin_config();//robot motion pins config
    left_encoder_pin_config(); //left encoder pin config
    right_encoder_pin_config(); //right encoder pin config
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
 	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
}
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
   motion_set(0x06);
}

void back (void) //both wheels backward
{
   motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
   motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void stop (void)
{
  motion_set(0x00);
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0;
 ShaftCountLeft = 0;

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 }
 stop(); //Stop robot
}



//Initialize the ports
//TIMER1 initialization in 10 bit fast PWM mode
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void servo_1(unsigned char degrees)
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}


void update_orientation(unsigned char data)
{
if (data==0x34 || data==0x61)
	orientation = 1;
if (data == 0x32 || data==0x62)
	orientation = 3;

if (data==0x33 || data== 0x63)
	orientation =0;
if (data == 0x31 || data==0x64)
	orientation = 2;
}



void run(unsigned char data)
{
		velocity(255,249);
		if((data == 0x34 && orientation==0) || (data == 0x32 && orientation==2) || (data == 0x33 && orientation==3) || (data == 0x31 && orientation==1)) //ASCII value of 8
		{   forward_mm(35);
			_delay_ms(50);
		    right_degrees(90);
			_delay_ms(50);
            back_mm(35);
			_delay_ms(50);
			stop();
			_delay_ms(100);
			forward_mm(190);
			stop();
			_delay_ms(100);	
			update_orientation(data);		
		}
        else if((data == 0x34 && orientation==2) || (data == 0x32 && orientation==0) || (data == 0x33 && orientation==1) || (data == 0x31 && orientation==3))
			{
			forward_mm(30);
			_delay_ms(50);
		    left_degrees(88);
			_delay_ms(50);
			back_mm(30);
			stop();
			_delay_ms(100);
			forward_mm(190);
			stop();
			_delay_ms(100);	
			update_orientation(data);		
		}
		else if((data == 0x34 && orientation==1)|| (data == 0x32 && orientation==3)|| (data == 0x33 && orientation==0)|| (data == 0x31 && orientation==2))
		{
			forward_mm(190);
			stop();
			_delay_ms(100);	
			update_orientation(data);		
		}
		else if((data == 0x34 && orientation==3) || (data == 0x32 && orientation==1) || (data == 0x33 && orientation==2) || (data == 0x31 && orientation==0))
		{
			forward_mm(30);
			_delay_ms(50);
		    left_degrees(177);
			_delay_ms(50);
			back_mm(30);
			stop();
			_delay_ms(100);
			forward_mm(180);
			stop();
			_delay_ms(100);	
			update_orientation(data);		
		}

	 if((data == 0x61 && orientation==0) || (data == 0x62 && orientation==2) || (data == 0x63 && orientation==3) || (data == 0x64 && orientation==1)) //ASCII value of 8
		{ 
		    forward_mm(35);
			_delay_ms(50);
		    right_degrees(90);
			_delay_ms(50);
			back_mm(35);
			_delay_ms(50);
			stop();
			_delay_ms(100);
			update_orientation(data);		
		}
        else if((data == 0x61 && orientation==2) || (data == 0x62 && orientation==0) || (data == 0x63 && orientation==1) || (data == 0x64 && orientation==3))
			{
			forward_mm(30);
			_delay_ms(50);
		    left_degrees(88);
			_delay_ms(50);
			back_mm(30);
			stop();
			_delay_ms(100);	
			update_orientation(data);		
		}
		else if((data == 0x61 && orientation==1)|| (data == 0x62 && orientation==3)|| (data == 0x63 && orientation==0)|| (data == 0x64 && orientation==2))
		{	
			//update_orientation(data);		
		}
		else if((data == 0x61 && orientation==3) || (data == 0x62 && orientation==1) || (data == 0x63 && orientation==2) || (data == 0x64 && orientation==0))
		{
		    forward_mm(30);
			_delay_ms(50);
		    left_degrees(177);
			_delay_ms(50);
			back_mm(30);
			stop();
			_delay_ms(100);
			update_orientation(data);		
		}
	

		if(data == 0x35) //ASCII value of 5
		{
			stop();//stop
		}

		if(data == 0x7a)
		{
			buzzer_on();
			_delay_ms(5000);
			buzzer_off();
		}

		//if(data == 0x39) //ASCII value of 9
		//{
		//	buzzer_off();
		//}

        if (data== 0x36)
		{
	//pickup_storage()
	  servo_3(120);
	  _delay_ms(500);
	  servo_1(90);
	  servo_2(0);
	  _delay_ms(250);
	  servo_1(15);
	  _delay_ms(50);
	  servo_2(180);
	  _delay_ms(250);
	  servo_1(90);
			
		}
		if(data == 0x37)
		{
	//pickup_buffer();
	   servo_3(120);
	  _delay_ms(500);
	  servo_1(90);
	  servo_2(0);
	  _delay_ms(250);
	  servo_1(15);
	  _delay_ms(50);
	  servo_2(90);

		}

		if(data == 0x38)
		{
	//drop_buffer();
      servo_3(120);
	  _delay_ms(100);
	  servo_1(15);
	  servo_2(0);
	  _delay_ms(150);
	  servo_1(90);

		}
		if(data== 0x39)
		{
    //drop_storage();
	  servo_3(120);
		servo_1(90);
		servo_2(180);
	  _delay_ms(250);
	   servo_1(15);
	  _delay_ms(50);
	   servo_2(0);
	  _delay_ms(250);
	    servo_1(90);
	  _delay_ms(500);
	  
	 
	  
	  
	  
		}

}
//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt) 
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	UDR0 = data;				//echo data back to PC
}


//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 uart0_init(); //Initailize UART1 for serial communiaction
 timer5_init();	//TIMER 5 for PWM CONTROL OF MOTORS
 timer1_init(); //TIMER 1 for SERVO PWM
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 port_init();
 sei(); //re-enable interrupts

}




//Main Function
int main(void)
{
	init_devices();
	while(1)
	{
	run(data);
	}
	return 0;
}
