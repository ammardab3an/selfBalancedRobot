
#define F_CPU 16000000UL	/* Define CPU clock Frequency e.g. here its 8MHz */

#include <avr/io.h>	/* Include AVR std. library file */
#include <util/delay.h>	/* Include delay header file */
#include <inttypes.h>	/* Include integer type header file */
#include <stdlib.h>	/* Include standard library file */
#include <stdio.h>	/* Include USART header file */
#include <util/twi.h>
#include <avr/interrupt.h>
#include "MPU6050_res_define.h"							/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"							/* Include I2C Master header file */
#include "USART_RS232_H_file.h"

#define RAD_TO_DEG 57.29577951308232

double dt, count;
double Acc_x, Acc_y, Acc_z, Gyro_x, Gyro_y, Gyro_z, Temperature;
double accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
double AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
double roll,pitch;

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
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()  //read the raw values of the sensor
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
	
	Acc_x = Acc_x/16384.0;	//Xa						/* Divide raw value by sensitivity scale factor to get real values */
	Acc_y = Acc_y/16384.0;	//Ya
	Acc_z= Acc_z/16384.0;	//Za
			
	Gyro_x = Gyro_x/16.4;	//Xg
	Gyro_y = Gyro_y/16.4;	//Yg
	Gyro_z = Gyro_z/16.4;	//Zg
}

void calculate_IMU_error(void) {
	
	int c = 0;
	while (c < 200) {
		
		Read_RawValue();
		
		AccErrorX = AccErrorX + ((atan((Acc_y) / sqrt(pow((Acc_x), 2) + pow((Acc_z), 2))) * RAD_TO_DEG));
		AccErrorY = AccErrorY + ((atan(-1 * (Acc_x) / sqrt(pow((Acc_y), 2) + pow((Acc_z), 2))) * RAD_TO_DEG));
		AccErrorZ = AccErrorZ + ((atan((Acc_z) / sqrt(pow((Acc_x), 2) + pow((Acc_z), 2))) * RAD_TO_DEG));

		GyroErrorX = GyroErrorX + Gyro_x;
		GyroErrorY = GyroErrorY + Gyro_y;
		GyroErrorZ = GyroErrorZ + Gyro_z;
		
		c++;
	}
	
	AccErrorX = AccErrorX / 200;
	AccErrorY = AccErrorY / 200;
	AccErrorZ = AccErrorZ / 200;

	GyroErrorX = GyroErrorX / 200;
	GyroErrorY = GyroErrorY / 200;
	GyroErrorZ = GyroErrorZ / 200;
}

void timer_setup(){
	TCCR1A = 0x00;
	TIMSK |= _BV(TOIE1);
	TCCR1B |= _BV(CS11);
	TCCR1B &= ~( _BV(CS12)  | _BV(CS10)); // prescaler=8
}

void get_time(double * dt){
	cli();
	uint8_t l = TCNT1L;
	uint8_t h = TCNT1H;
	uint16_t step = h<<8 | l;
	*dt = (double)step*5e-7 + count*0.032768;
	count = 0;
	sei();
}

SIGNAL(TIMER1_OVF_vect){
	count += 1;
	//TCNT1H = 0x00;
	//TCNT1L = 0x00;
}

int main()
{
	char buffer[20], double_[10];
	
	I2C_Init();
	Gyro_Init();
	I2C_Init();
	USART_Init(19200);
	timer_setup();
	calculate_IMU_error();
	
	while(1)
	{
		Read_RawValue();
		get_time(&dt);
		
		// Calculating Roll and Pitch from the accelerometer data
		accAngleX = (atan(Acc_y / sqrt(pow(Acc_x, 2) + pow(Acc_z, 2))) * RAD_TO_DEG) - AccErrorX;
		accAngleY = (atan(-1 * Acc_x / sqrt(pow(Acc_y, 2) + pow(Acc_z, 2))) * RAD_TO_DEG) - AccErrorY;
		accAngleZ = (atan(Acc_z / sqrt(pow(Acc_x, 2) + pow(Acc_z, 2))) * RAD_TO_DEG) - AccErrorZ;
		
		// Correct the outputs with the calculated error values
		Gyro_x = Gyro_x - GyroErrorX;
		Gyro_y = Gyro_y - GyroErrorY;
		Gyro_z = Gyro_z - GyroErrorZ;

		// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
		gyroAngleX = gyroAngleX + Gyro_x * dt; // deg/s * s = deg
		gyroAngleY = gyroAngleY + Gyro_y * dt;
		gyroAngleZ = gyroAngleZ + Gyro_z * dt;
		
		// Complementary filter - combine acceleromter and gyro angle values
		gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
		gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;
		
		roll = gyroAngleX;
		pitch = gyroAngleY;
		
		dtostrf(roll, 3, 2, double_);
		sprintf(buffer, "%s/", double_);
		USART_SendString(buffer);
		
		dtostrf(pitch, 3, 2, double_);
		sprintf(buffer, "%s\n", double_);
		USART_SendString(buffer);		
	}
}
