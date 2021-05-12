
#define F_CPU 16000000UL	/* Define CPU clock Frequency e.g. here its 8MHz */

#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "MPU6050_res_define.h"
#include "I2C_Master_H_file.h"
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
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);
	I2C_Write(0x18);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(ACCEL_CONFIG);							/* Write to Accelo configuration register */
	I2C_Write(0x00);									/* Full scale range +/- 2g */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(MOT_THR);									/* Write to motion threshold register */
	I2C_Write(0x00);									/* Motion detection threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FIFO_EN);									/* Write to FIFO enable register */
	I2C_Write(0x00);									/* FIFO disabled */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FF_THR);									/* Write to free fall threshold register */
	I2C_Write(0x00);									/* Free fall threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FF_DUR);									/* Write to free fall duration register */
	I2C_Write(0x00);									/* Free fall duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(MOT_DUR);									/* Write to motion duration register */
	I2C_Write(0x00);									/* Motion detection duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(ZRMOT_DUR);								/* Write to zero motion duration register */
	I2C_Write(0x00);									/* Zero motion detection duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(ZRMOT_THR);								/* Write to zero motion threshold register */
	I2C_Write(0x00);									/* Zero motion detection threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_MST_CTRL);							/* Write to I2C Master control register */
	I2C_Write(0x00);									/* Disable multi-master */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_REG);							/* Write to I2C Slave0 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_ADDR);							/* Write to I2C Slave0 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_CTRL);							/* Write to I2C Slave0 Control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_REG);							/* Write to I2C Slave1 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_ADDR);							/* Write to I2C Slave1 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_CTRL);							/* Write to I2C Slave1 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_REG);							/* Write to I2C Slave2 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_REG);							/* Write to I2C Slave2 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_CTRL);							/* Write to I2C Slave2 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_REG);							/* Write to I2C Slave3 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_ADDR);							/* Write to I2C Slave3 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_CTRL);							/* Write to I2C Slave3 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_REG);							/* Write to I2C Slave4 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_ADDR);							/* Write to I2C Slave4 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_CTRL);							/* Write to I2C Slave4 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_DO);								/* Write to I2C Slave4 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_DI);								/* Write to I2C Slave4 data in register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_PIN_CFG);								/* Write to interrupt pin configuration register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_DO);								/* Write to I2C Slave0 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_DO);								/* Write to I2C Slave1 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_DO);								/* Write to I2C Slave2 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_DO);								/* Write to I2C Slave3 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_MST_DELAY_CTRL);						/* Write to I2C Master delay control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(SIGNAL_PATH_RESET);						/* Write to Signal Path Reset register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(MOT_DETECT_CTRL);							/* Write to Motion detection control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(USER_CTRL);								/* Write to User control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_2);								/* Write to power management register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FIFO_R_W);								/* Write to FIFO R/W register */
	I2C_Write(0x00);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */
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
	Acc_z = Acc_z/16384.0;	//Za
	
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
	TCCR1B &= ~(_BV(CS12)  | _BV(CS10)); // prescaler=8
}

void get_time_sec(double* dt){
	uint8_t sreg = SREG;
	cli();
	*dt = (double)TCNT1*5e-7 + count*0.032768; // step time = (prescaler=8) * 1.0/(f_cpu=16m), count = (max timer value=0xffff)) * (step time)
	count = TCNT1 = 0;
	SREG = sreg;
}

SIGNAL(TIMER1_OVF_vect){
	count += 1;
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
		get_time_sec(&dt);
		
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
		sprintf(buffer, "%s/", double_);
		USART_SendString(buffer);
		
		dtostrf(dt, 3, 2, double_);
		sprintf(buffer, "%s\n", double_);
		USART_SendString(buffer);
	}
}
