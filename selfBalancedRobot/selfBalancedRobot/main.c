
#define F_CPU 16000000UL	/* Define CPU clock Frequency e.g. here its 8MHz */

#include <avr/io.h>	/* Include AVR std. library file */
#include <util/delay.h>	/* Include delay header file */
#include <inttypes.h>	/* Include integer type header file */
#include <stdlib.h>	/* Include standard library file */
#include <stdio.h>	/* Include USART header file */
#include <util/twi.h>

#define I2C_TIMER_DELAY 0xFF
#define SCL_CLOCK  400000L
#define BAUD_PRESCALE (((F_CPU / (19200 * 16UL))) - 1)
#define TW_START		0x08
#define TW_REP_START		0x10

extern void I2C_init(void);
extern void I2C_Stop(void);
extern unsigned char I2C_Start(unsigned char addr);
extern unsigned char I2C_Rep_Start(unsigned char addr);
extern void I2C_Start_Wait(unsigned char addr);
extern unsigned char I2C_Write(unsigned char data);
extern unsigned char I2C_ReadAck(unsigned char ack);
extern unsigned char I2C_ReadNak(unsigned char nack);
extern unsigned char I2C_Read(unsigned char ack);

void I2C_init(void)
{
	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	
	TWSR = 0;                         /* no prescaler */
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}

unsigned char I2C_Start(unsigned char address)
{
	uint32_t  i2c_timer = 0;
	uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
	return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
	return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;
}

void I2C_Start_Wait(unsigned char address)
{
	uint32_t  i2c_timer = 0;
	uint8_t   twst;

	while ( 1 )
	{
		// send START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		
		// wait until transmission completed
		i2c_timer = I2C_TIMER_DELAY;
		while(!(TWCR & (1<<TWINT)) && i2c_timer--);

		// check value of TWI Status Register. Mask prescaler bits.
		twst = TW_STATUS & 0xF8;
		if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
		
		// send device address
		TWDR = address;
		TWCR = (1<<TWINT) | (1<<TWEN);
		
		// wail until transmission completed
		i2c_timer = I2C_TIMER_DELAY;
		while(!(TWCR & (1<<TWINT)) && i2c_timer--);
		
		// check value of TWI Status Register. Mask prescaler bits.
		twst = TW_STATUS & 0xF8;
		if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ){
			/* device busy, send stop condition to terminate write operation */
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
			
			// wait until stop condition is executed and bus released
			i2c_timer = I2C_TIMER_DELAY;
			while((TWCR & (1<<TWSTO)) && i2c_timer--);
			continue;
		}
		break;
	}
}

unsigned char I2C_Rep_Start(unsigned char address){	
	return I2C_Start( address );  
}

void I2C_Stop(void)
{
	uint32_t  i2c_timer = 0;

	/* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	i2c_timer = I2C_TIMER_DELAY;
	while((TWCR & (1<<TWSTO)) && i2c_timer--);

}

unsigned char I2C_Write( unsigned char data )
{
	uint32_t  i2c_timer = 0;
	uint8_t   twst;
	
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
	return 1;

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;
}

unsigned char I2C_ReadAck(unsigned char ack)
{
	uint32_t  i2c_timer = 0;

	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
	return 0;

	return TWDR;
}

unsigned char I2C_ReadNak(unsigned char ack)
{
	uint32_t  i2c_timer = 0;

	TWCR = (1<<TWINT) | (1<<TWEN);
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
	return 0;
	
	return TWDR;
}

float kalman = 0, kalmanX = 0, kalmanY = 0, dt = 0;
float Acc_x, Acc_y, Acc_z, Gyro_x, Gyro_y, Gyro_z;

void Gyro_Init()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(0x19);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x6B);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x1A);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x1B);
	I2C_Write(0x18);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x1C);							/* Write to Accelo configuration register */
	I2C_Write(0x00);									/* Full scale range +/- 2g */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x1F);									/* Write to motion threshold register */
	I2C_Write(0x00);									/* Motion detection threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x23);									/* Write to FIFO enable register */
	I2C_Write(0x00);									/* FIFO disabled */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x1D);									/* Write to free fall threshold register */
	I2C_Write(0x00);									/* Free fall threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x1E);									/* Write to free fall duration register */
	I2C_Write(0x00);									/* Free fall duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0X20);									/* Write to motion duration register */
	I2C_Write(0x00);									/* Motion detection duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x22);								/* Write to zero motion duration register */
	I2C_Write(0x00);									/* Zero motion detection duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x21);								/* Write to zero motion threshold register */
	I2C_Write(0x00);									/* Zero motion detection threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x24);							/* Write to I2C Master control register */
	I2C_Write(0x00);									/* Disable multi-master */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x26);							/* Write to I2C Slave0 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x25);							/* Write to I2C Slave0 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x27);							/* Write to I2C Slave0 Control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x29);							/* Write to I2C Slave1 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x28);							/* Write to I2C Slave1 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x2A);							/* Write to I2C Slave1 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x2C);							/* Write to I2C Slave2 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x2B);							/* Write to I2C Slave2 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x2D);							/* Write to I2C Slave2 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x2F);							/* Write to I2C Slave3 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x2E);							/* Write to I2C Slave3 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x30);							/* Write to I2C Slave3 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x32);							/* Write to I2C Slave4 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x31);							/* Write to I2C Slave4 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x34);							/* Write to I2C Slave4 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x33);								/* Write to I2C Slave4 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x35);								/* Write to I2C Slave4 data in register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x37);								/* Write to interrupt pin configuration register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x38);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x63);								/* Write to I2C Slave0 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x64);								/* Write to I2C Slave1 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x65);								/* Write to I2C Slave2 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x66);								/* Write to I2C Slave3 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x67);						/* Write to I2C Master delay control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x68);						/* Write to Signal Path Reset register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x69);							/* Write to Motion detection control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x6A);								/* Write to User control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x6C);								/* Write to power management register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(0x74);								/* Write to FIFO R/W register */
	I2C_Write(0x00);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(0x3B);							/* Write start location address from where to read */
	I2C_Rep_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()  //read the raw values of the sensor
{
	MPU_Start_Loc();

  //Read Accelerometer values
	Acc_x = ~((((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadAck(0xd2))-1);
	Acc_y = ~((((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadAck(0xd2))-1);
	Acc_z = ~((((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadAck(0xd2))-1);
	
  //Temperature = (((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadAck(0x69)); //uncomment to read temperature values
	
  //Read Gyro values
	Gyro_x = ~((((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadAck(0xd2))-1);
	Gyro_y = ~((((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadAck(0xd2))-1);
	Gyro_z = ~((((int)I2C_ReadAck(0xd2)<<8) | (int)I2C_ReadNak(0))-1);
  
	I2C_Stop(); //stop I2C serial communication
}

void USART_init() //initialize the USART
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);   /* Turn on the transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); /* Use 8-bit character sizes */

	UBRRL = BAUD_PRESCALE; 				  /* Load lower 8-bits of the baud rate value */
	UBRRH = (BAUD_PRESCALE >> 8);		  /*Load upper 8-bits*/
}

unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0); /*Do nothing until data have been received*/
	return(UDR);				       /* return the byte*/
}

void UART_TxChar(char ch)
{
	while (! (UCSRA & (1<<UDRE)));     /*Wait for empty transmit buffer*/
	UDR = ch ;
}

//function to send string to the USART
void USART_SendString(char *str)
{
	unsigned char j=0;
	
	while (str[j]!=0)	               /*send string up to null */
	{
		UART_TxChar(str[j]);
		j++;
	}
}

//kalman filter parameter for X
double q_angle=0.008,q_bias=0.003,r_measure=0.003;
double angleX=0,biasX=0,rateX;
double pX[2][2]={{0,0},{0,0}},kX[2],yX,sX;

//Kalman filter parameter for Y
double angleY=0,biasY=0,rateY;
double pY[2][2]= {{0,0},{0,0}},kY[2],yY,sY;

float kalman_get_angle_X(float newAngle, float newRate, float dt)
{
	rateX = newRate-biasX;
	angleX += dt*rateX;

	//update error estimation error covariance
	pX[0][0] += dt * (dt*pX[1][1]-pX[0][1]-pX[1][0]+q_angle);
	pX[0][1] -= dt * pX[1][1];
	pX[1][0] -= dt * pX[1][1];
	pX[1][1] += q_bias * dt;

	//calculate kalman gain
	sX = pX[0][0] + r_measure;

	kX[0] = pX[0][0]/sX;
	kX[1] = pX[1][0]/sX;

	yX=newAngle-angleX;
	angleX+=kX[0]*yX;
	biasX+=kX[1]*yX;

	//Calculate estimation error covariance
	pX[0][0]-=kX[0]*pX[0][0];
	pX[0][1]-=kX[0]*pX[0][1];
	pX[1][0]-=kX[1]*pX[0][0];
	pX[1][1]-=kX[1]*pX[0][1];

	return angleX;
}

float kalman_get_angle_Y(float newAngle, float newRate, float dt)
{
	rateY = newRate-biasY;
	angleY += dt*rateY;

	//update error estimation error covariance
	pY[0][0] += dt * (dt*pY[1][1]-pY[0][1]-pY[1][0]+q_angle);
	pY[0][1] -= dt * pY[1][1];
	pY[1][0] -= dt * pY[1][1];
	pY[1][1] += q_bias * dt;

	//calculate kalman gain
	sY = pY[0][0] + r_measure;

	kY[0] = pY[0][0]/sY;
	kY[1] = pY[1][0]/sY;

	yY=newAngle-angleY;
	angleY+=kY[0]*yY;
	biasY+=kY[1]*yY;

	//Calculate estimation error covariance
	pY[0][0]-=kY[0]*pY[0][0];
	pY[0][1]-=kY[0]*pY[0][1];
	pY[1][0]-=kX[1]*pY[0][0];
	pY[1][1]-=kX[1]*pY[0][1];

	return angleY;
}

float roll,pitch,prev_gyroX,prev_gyroY,prev_gyroZ,prev_kalmanX,prev_kalmanY,Gyro_Rx,Gyro_Ry,gyroAngleX,gyroAngleY;

int main()
{
	char buffer[20], float_[10];
	
	I2C_init();											/* Initialize I2C */
	Gyro_Init();										/* Initialize Gyro */
	USART_init();									/* Initialize USART with 9600 baud rate */
	
	//DDRB = 0XFF;
//
	//DDRD=0x80; /* PD7 pin of PORTD is declared output (PWM2 pin of DC Motor Driver is connected) */
	//DDRA=0x0f; /*PC0,PC1,PC2 and PC3 pins of PortC are declared output ( i/p1,i/p2,i/p3 and i/p4 pins of DC Motor Driver areconnected)*/
	//TCNT0=0xf8;
	//TCNT2=0xf8;
	//TCCR0=0x6C; // fast pwm mode,presclar 256,frequency set to 3.90625,non-inverting mode
	//TCCR2=0x6C;
	
	while(1)
	{
		Read_RawValue();

		Acc_x = Acc_x/16384.0;	//Xa						/* Divide raw value by sensitivity scale factor to get real values */
		Acc_y = Acc_y/16384.0;	//Ya
		Acc_z= Acc_z/16384.0;	//Za
		
		Gyro_x = Gyro_x/16.4;	//Xg
		Gyro_y = Gyro_y/16.4;	//Yg
		Gyro_z = Gyro_z/16.4;	//Zg

		dt=0.011;

		roll=atan(Acc_x/sqrt(Acc_x*Acc_x + Acc_z*Acc_z))*57.324;
		pitch=atan(-Acc_x/sqrt(Acc_y*Acc_y + Acc_z*Acc_z))*57.324;

		angleX=roll;
		angleY=pitch;

		prev_gyroX=Gyro_x;
		prev_gyroY=Gyro_y;
		prev_gyroZ=Gyro_z;

		prev_kalmanX=kalmanX;
		prev_kalmanY=kalmanY;

		Gyro_Rx=Gyro_x;
		Gyro_Ry=Gyro_y;

		//calculating kalman angle.This fixes the transition problemwhen accerlerometer angle jumps between -180
		if((roll<-90 && kalmanX > 90) || (roll>90 && kalmanX < -90))
		{
			angleX = roll;
			kalman = roll;
		}
		else kalmanX=kalman_get_angle_X(roll,Gyro_Rx,dt);
		
		if(abs(kalmanX)>90)
		Gyro_Ry=-Gyro_Rx;
		kalmanY = kalman_get_angle_Y(pitch,Gyro_Ry,dt);

		if((pitch<-90 && kalmanY>90) || (pitch>90 && kalmanY<-90))
		{
			angleY = pitch;
			kalmanY=pitch;
		}
		else kalmanY = kalman_get_angle_Y(pitch,Gyro_Ry,dt);

		if(abs(kalmanY)>90)
		Gyro_Rx = -Gyro_Rx;
		kalmanX=kalman_get_angle_X(roll,Gyro_Rx,dt);
		//("kalmanX angle:%f\t\tkalmanY angle:%f\n\n");//,kalmanX,kalmanY);
		
		gyroAngleX =(prev_kalmanX-kalmanX)*0.9;
		gyroAngleY =(prev_kalmanY-kalmanY)*0.5;
		Gyro_z = Gyro_z*0.05;

		//Threshold sensor value
		if(gyroAngleX < 0.6 && gyroAngleX > -0.6)
		gyroAngleX=0;
		if(gyroAngleY<0.6 && gyroAngleY > -0.6)
		gyroAngleY=0;

		dtostrf(gyroAngleX, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
		sprintf(buffer,"gyroAngleX = %s g    ",float_);
		USART_SendString(buffer);

		dtostrf(gyroAngleY, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
		sprintf(buffer,"gyroAngleY = %s g\r\n",float_);
		USART_SendString(buffer);

		//if(gyroAngleY>0){
			//PORTA=0x0a;
			//_delay_us(2000);
			//PORTA=0X05;
			//_delay_us(1000);
		//}
		//
		//OCR0=180;
		//OCR2=180;
		//OCR0=100;	/*OCR2 register value is set to 100*/
		//OCR2=100; /*OCR2 register value is set to 100*/
	}
}
