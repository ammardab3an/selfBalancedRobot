
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include "MPU6050_res_define.h"
#include "I2C_Master_H_file.h"
#include "USART_RS232_H_file.h"

#define RAD_TO_DEG 57.29577951308232

double dt, count;
double Acc_x, Acc_y, Acc_z, Gyro_x, Gyro_y, Gyro_z, Temperature;
double accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
double AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
double roll,pitch;
double error, sum_error, pre_error, motor_power;
double target = 0.0;
double Kp, Ki, Kd;
uint8_t go_flag, usart_pre;

void PWM_Init();
void PWM_SetDutyCycle(double dutyCycle);
void PWM_Start();
void Gyro_Init();
void MPU_Start_Loc();
void Read_RawValue();
void calculate_IMU_error(void);
void timer_setup();
void get_time_sec();
SIGNAL(TIMER1_OVF_vect);
void ADC_Init();
uint16_t ADC_GetAdcValue(uint8_t v_adcChannel_u8);

void Gyro_Init()
{
	_delay_ms(150); 	    /* Power up time >100ms */
	
	I2C_Start_Wait(0xD0); 	/* Start with device write address */
	I2C_Write(SMPLRT_DIV); 	/* Write to sample rate register */
	I2C_Write(0x07); 		/* 8k/(7+1) = 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1); 	/* Write to power management register */
	I2C_Write(0x01); 		/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG); 		/* Write to Configuration register */
	I2C_Write(0x02); 		/* acc_bandwidth = 94, gyro_bandwidth = 98, Fs = 1KHz, dealy_gyro = 3ms, delay_acc = 2.8ms*/
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG); /* full scal range ± 250 °/s */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(ACCEL_CONFIG); /* Write to Accelo configuration register */
	I2C_Write(0x00); 		 /* Full scale range +/- 2g */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(MOT_THR); 	 /* Write to motion threshold register */
	I2C_Write(0x00); 		 /* Motion detection threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FIFO_EN); 	 /* Write to FIFO enable register */
	I2C_Write(0x00); 		 /* FIFO disabled */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FF_THR); 		/* Write to free fall threshold register */
	I2C_Write(0x00); 		/* Free fall threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FF_DUR); 		/* Write to free fall duration register */
	I2C_Write(0x00); 		/* Free fall duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(MOT_DUR); 	/* Write to motion duration register */
	I2C_Write(0x00); 		/* Motion detection duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(ZRMOT_DUR); 	/* Write to zero motion duration register */
	I2C_Write(0x00); 		/* Zero motion detection duration counter */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(ZRMOT_THR); 	/* Write to zero motion threshold register */
	I2C_Write(0x00); 		/* Zero motion detection threshold value */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_MST_CTRL); /* Write to I2C Master control register */
	I2C_Write(0x00); 		 /* Disable multi-master */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_REG);  /* Write to I2C Slave0 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_ADDR); /* Write to I2C Slave0 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_CTRL); /* Write to I2C Slave0 Control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_REG); /* Write to I2C Slave1 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_ADDR); /* Write to I2C Slave1 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_CTRL); /* Write to I2C Slave1 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_REG); /* Write to I2C Slave2 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_REG); /* Write to I2C Slave2 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_CTRL); /* Write to I2C Slave2 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_REG); /* Write to I2C Slave3 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_ADDR); /* Write to I2C Slave3 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_CTRL); /* Write to I2C Slave3 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_REG); /* Write to I2C Slave4 data register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_ADDR); /* Write to I2C Slave4 address register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_CTRL); /* Write to I2C Slave4 control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_DO);   /* Write to I2C Slave4 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV4_DI);   /* Write to I2C Slave4 data in register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_PIN_CFG);   /* Write to interrupt pin configuration register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE); 	/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV0_DO); 	/* Write to I2C Slave0 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV1_DO); 	/* Write to I2C Slave1 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV2_DO); 	/* Write to I2C Slave2 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_SLV3_DO); 	/* Write to I2C Slave3 data out register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(I2C_MST_DELAY_CTRL);	/* Write to I2C Master delay control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(SIGNAL_PATH_RESET);	/* Write to Signal Path Reset register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(MOT_DETECT_CTRL); /* Write to Motion detection control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(USER_CTRL); 	/* Write to User control register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_2); 	/* Write to power management register */
	I2C_Write(0x00);
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(FIFO_R_W); 	/* Write to FIFO R/W register */
	I2C_Write(0x00);
	I2C_Stop();
}


void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0); 	  /* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);  /* Write start location address from where to read */
	I2C_Repeated_Start(0xD1); /* I2C start with device read address */
}

void Read_RawValue()  //read the raw values of the sensor
{
	MPU_Start_Loc();
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
	
	/* Divide raw value by sensitivity scale factor to get real values */
	Acc_x = Acc_x/16384.0;	//Xa
	Acc_y = Acc_y/16384.0;	//Ya
	Acc_z = Acc_z/16384.0;	//Za
	
	Gyro_x = Gyro_x/131.0;	//Xg
	Gyro_y = Gyro_y/131.0;	//Yg
	Gyro_z = Gyro_z/131.0;	//Zg
}

void calculate_IMU_error(void) {
	
	int c = 0;
	while (c < 200) {
		
		Read_RawValue();
		get_time_sec();
		
		AccErrorX = AccErrorX + ((atan((Acc_y) / sqrt(pow((Acc_x), 2) + pow((Acc_z), 2))) * RAD_TO_DEG));
		AccErrorY = AccErrorY + ((atan(-1 * (Acc_x) / sqrt(pow((Acc_y), 2) + pow((Acc_z), 2))) * RAD_TO_DEG));
		AccErrorZ = AccErrorZ + ((atan((Acc_z) / sqrt(pow((Acc_x), 2) + pow((Acc_z), 2))) * RAD_TO_DEG));

		GyroErrorX = GyroErrorX + Gyro_x * dt; // deg/s * s = deg
		GyroErrorY = GyroErrorY + Gyro_y * dt;
		GyroErrorZ = GyroErrorZ + Gyro_z * dt;
		
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
	TIMSK |= 1 << TOIE1;
	TCCR1B |= 1 << CS11;
	TCCR1B &= ~((1 << CS12) | (1 << CS10)); // prescaler = 8
}

void get_time_sec(){
	double cur = TCNT1;
	// step time = (prescaler=8) * 1.0/(f_cpu=16m), count = (max timer value=0xffff)) * (step time)
	dt = (5e-7) * cur + (0.032768) * count; 
	// don't forget to reset tcnt1 also, because we used its value in our calculation.
	count = TCNT1 = 0; 
}

SIGNAL(TIMER1_OVF_vect){
	count += 1;
}

void PWM_Init()
{
	TCNT0 = 0x00;
	TCNT2 = 0X00;
	DDRB |= 1 << PB3;
	DDRD |= 1 << PD7;
}

void PWM_Start()
{
	TCCR0 = (1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS00); // maximum pwm freq using fast pwm mode without a prescaler
	TCCR2 = (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<CS20); // 16m / 256 = 62.5khz
}

void PWM_SetDutyCycle(double dutyCycle)
{
	
	dutyCycle = 2.55 * dutyCycle;
	
	double l_val, r_val;
	
	l_val = 1.08 * dutyCycle;
	r_val = dutyCycle;
	
	if(l_val > 255) l_val = 255;
	if(r_val > 255) r_val = 255;
	
	OCR0 = (uint8_t) l_val;
	OCR2 = (uint8_t) r_val;
}

void ADC_Init()
{
	/* Result right justified, select channel zero */
	ADMUX = (1 << REFS0);
	/* Enable ADC , sampling freq=osc_freq/64 = 250khz*/
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS2); 
}

uint16_t ADC_GetAdcValue(uint8_t v_adcChannel_u8)
{
	
	ADMUX = (1 << REFS0) | v_adcChannel_u8;   /* Select the required channel */
	_delay_us(10);                            /* Wait for some time for the channel to get selected */
	ADCSRA |= 1 << ADSC;                      /* Start the ADC conversion by setting ADSC bit */
	
	while((ADCSRA & (1 << ADIF)) == 0);       /* Wait till the conversion is over */
	/* ADIF will be set once ADC conversion is complete */
	return(ADCW);                             /* Return the 10-bit result */
}

int main()
{
	
	DDRB |= (1<<PB0) | (1<<PB1);
	DDRD |= (1<<PD5) | (1<<PD6);
	
	I2C_Init();
	USART_Init(57600);
	
	timer_setup();
	sei();
	
	Gyro_Init();
	calculate_IMU_error();

	ADC_Init();
	
	PWM_Init();
	PWM_SetDutyCycle(0);
	PWM_Start();
	
	char buffer[20], _double[10];
	
	while(1)
	{
		Read_RawValue();
		get_time_sec();
		
		// Calculating Roll and Pitch from the accelerometer data
		accAngleX = (atan(Acc_y / sqrt(pow(Acc_x, 2) + pow(Acc_z, 2))) * RAD_TO_DEG) - AccErrorX;
		accAngleY = (atan(-1.0 * Acc_x / sqrt(pow(Acc_y, 2) + pow(Acc_z, 2))) * RAD_TO_DEG) - AccErrorY;
		
		// Correct the outputs with the calculated error values
		gyroAngleX = gyroAngleX + Gyro_x * dt - GyroErrorX; 
		gyroAngleY = gyroAngleY + Gyro_y * dt - GyroErrorY;
		
		// Complementary filter - combine accelerometer and gyro angle values
		gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
		gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;
		
		roll = gyroAngleX;
		pitch = gyroAngleY;
		
		uint16_t adc[4];
		for(int i = 0; i < 4; i++){
			adc[i] = ADC_GetAdcValue(i);
		}
		
		Kp = (double) adc[1] * (10.0 / 1024.0);
		Ki = (double) adc[2] * (6.0 / 1024.0);
		Kd = (double) adc[3] * (0.3 / 1024.0);
		
		pitch += (double) 65 + adc[0] * (15.0 / 1024.0);
		
		error = pitch - target;
		
		go_flag |= (-10 < error) && (error < 10);
		
		if(!go_flag || (pitch > 50) || (pitch < -50) || (roll > 50) || (roll < -50)){
			PWM_SetDutyCycle(0);
			PORTB |= (1<<PB1);
			PORTB &= ~(1<<PB0);
		}
		else{
			
			PORTB |= (1<<PB0);
			PORTB &= ~(1<<PB1);
			
			sum_error += error * dt;
			
			double mx_val = 20;
			if(sum_error > mx_val)
			sum_error = mx_val;
			if(sum_error < -mx_val)
			sum_error = -mx_val;
			
			motor_power = Kp * error + Ki * sum_error + Kd * (error - pre_error) / dt;

			pre_error = error;
			
			if(motor_power > 0){
				PORTD |= 1 << 5;
				PORTD &= ~(1 << 6);
			}
			else{
				PORTD &= ~(1 << 5);
				PORTD |= 1 << 6;
				motor_power *= -1;
			}
			
			motor_power += 10;
			
			if(motor_power > 100){
				motor_power = 100;
			}
			
			PWM_SetDutyCycle(motor_power);
		}
		
		//............................................................................//
		
		usart_pre += 1;
		usart_pre %= 1;
		
		if(usart_pre == 0){
			
			double tmp[] = {
				pitch,
				Kp, Ki, Kd,
				sum_error, motor_power,
				dt
			};
			
			int tmp_sz = sizeof(tmp) / sizeof(tmp[0]);
			
			for(int i = 0; i < tmp_sz; i++){
				dtostrf(tmp[i], 3, 3, _double);
				sprintf(buffer, "%s/", _double);
				USART_SendString(buffer);
			}
			USART_SendString("\n");
			
		}
		else{
			// added delay to set the sampling frequency
			_delay_ms(3);
		}
		
	}
}
