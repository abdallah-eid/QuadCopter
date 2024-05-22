/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * AUTHOR			: MOUSTAFA ABUELMAGD & Boda
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */

#include <stdint.h>
#include <stdio.h>
#include "stm32f103xb.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>




uint32_t duration;
float distance;
//prototypes of the used functions
//void delayuS(uint32_t us);
uint32_t read_echo(uint32_t timeout);
void delayuS(uint32_t us);



float theta_pitch_new = 0;
float uncertainty = 0;
float theta_roll_new = 0;
float theta_roll_uncertainty = 0;
#define Trig_high               GPIOB->BSRR = GPIO_BSRR_BS8                      // turn on       PA0 (trig pin)
#define Trig_low                GPIOB->BSRR = GPIO_BSRR_BR8                      // turn off      PA0 (trig pin)



int16_t readings = 0;
int16_t gyroy_prev = 0;
int16_t acc_x_calibration = 0;
int16_t acc_y_calibration = 0;
int16_t acc_z_calibration = 0;
int16_t gyro_x_calibration = 0;
int16_t gyro_y_calibration = 0;
int16_t gyro_z_calibration = 0;

float kalman_angle_roll = 0;
float kalman_uncertainty_roll = 2*2;

float kalman_angle_pitch = 0;
float kalman_uncertainty_pitch = 2*2;

float kalman_values[] = {0,0};
float kalman_values_roll[] = {0,0};

float theta_pitch_gyro = 0;
int16_t theta_pitch;
int16_t theta_roll;
float	acc_x;
float	acc_y;
float	acc_z;
int16_t	gyro_x;
int16_t	gyro_y;
int16_t	gyro_z;

void kalman_filter_roll(float state, float uncertainty, float input_gyro, float measurment_acc);
void kalman_filter(float state, float uncertainty, float input_gyro, float measurment_acc);
int16_t gyro_angle(float state, float input);
int16_t imu_cal(int sensor,float sensitivity);
float read_imu(int sensor,float sensitivity);
void signal_lpfilter();
void imu_awak();
void init_i2c();
void delay(int d);

void task_motor_1(void *pvParameter)
{
		while(1){
			TIM3->CCR1 = 5332;
		}
}

void task_motor_2(void *pvParameter)
{
		while(1){
			TIM2->CCR2 = 5332;
		}
}

void task_motor_3(void *pvParameter)
{
		while(1){
			TIM2->CCR3 = 2800 ;
		}
}
void task_motor_4(void *pvParameter)
{
		while(1){
   			TIM2->CCR4 = 5332;
		}
}
/*void task_ultrasonic(void *pvparameter){
//	    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                                          //enable GPIOA Clock
//	    GPIOA->CRL &= ~GPIO_CRL_MODE0;                                                       // Clear mode bits for PA0
//	    GPIOA->CRL |= GPIO_CRL_MODE0_1;                                                      // Set PA0 to Output max speed 2 MHz
//	    GPIOA->CRL &= ~GPIO_CRL_CNF0;                                                        // Set PA0 to General purpose output push-pull

	    //configure Timer2 to generate microseconds delay
//	    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                                                  // Enable TIM2 clock
//	    TIM2->PSC = 72 - 1;                                                                                  // 72 MHz / 72 = 1 MHz (1 microsecond per tick)
//	    TIM2->ARR = 1;                                                                                       // Auto-reload register value
//	    TIM2->CNT = 0;
//	    TIM2->CR1 = TIM_CR1_CEN;                                                                     // Enable the counter
 while(1){
		GPIOB->ODR &=~(1<<8);												// turn off trig
		delayuS(10); 											// wait 10 us
	    GPIOB->ODR |=(1<<8);  											// turn on trig
		delayuS(10); 											// wait 10 us
		GPIOB->ODR &=~(1<<8);												// turn off trig
		duration = read_echo(1000000); 							// measure the time of echo pin
		distance = duration* 0.342 ;							// distance = duration / 2 * SOUND_SPEED
		if(distance != 0){
			GPIOB->ODR |= (1<<10);
		}
		else{
			GPIOB->ODR &=~(1<<10);
		}
		vTaskDelay(pdMS_TO_TICKS(50));
 }
}*/
void task_imu(void *pvParameter){
		while(1){//round(value * 100) / 100.0
			float valz = read_imu(59,2048.0) - acc_z_calibration ;
			float valy = read_imu(61,2048.0) + 0.13 ;
			float valx = read_imu(63,2048.0) + 0.13 ;
			acc_z =  -(round(valz*100))/100;
			acc_y = -(round(valy*100))/100;
			acc_x =  -(round(valx*100)/100);
			gyro_z = (read_imu(67,16.4)) - gyro_x_calibration;
			gyro_y = (read_imu(69,16.4)) - gyro_y_calibration;
			gyro_x = (read_imu(71,16.4)) - gyro_z_calibration;

			theta_roll = atan( acc_y/( sqrt(acc_z*acc_z+acc_x*acc_x) ) )*(180/M_PI);
			theta_pitch = atan( -acc_x/( sqrt(acc_z*acc_z+acc_y*acc_y) ) )*(180/M_PI);
			theta_pitch_gyro = gyro_angle(theta_pitch_gyro, gyro_y);


			kalman_filter(theta_pitch_new, uncertainty, gyro_y, theta_pitch);

			theta_pitch_new = kalman_values[0];
			uncertainty = kalman_values[1];

			kalman_filter_roll(kalman_values_roll[0],kalman_values_roll[1],gyro_x,theta_roll);

			theta_roll_new = kalman_values_roll[0];
			theta_roll_uncertainty = kalman_values_roll[1];

			// accelerometer sensitivity 2048
			// gyroscope sensitivity 16.4
			GPIOB->ODR &=~(1<<8);												// turn off trig
			delayuS(10); 											// wait 10 us
			GPIOB->ODR |=(1<<8);  											// turn on trig
			delayuS(10); 											// wait 10 us
			GPIOB->ODR &=~(1<<8);												// turn off trig
			duration = read_echo(1000000); 							// measure the time of echo pin
			distance = duration* 0.342 ;							// distance = duration / 2 * SOUND_SPEED
			if(distance != 0){
				GPIOB->ODR |= (1<<10);
			}
			else{
				GPIOB->ODR &=~(1<<10);
			}
			//vTaskDelay(pdMS_TO_TICKS(50));
			vTaskDelay(pdMS_TO_TICKS(50));
		}
}

int main(void){
		int16_t acc_x_calibration = 0;
		int16_t acc_y_calibration = 0;
		int16_t acc_z_calibration = 0;
		int16_t gyro_x_calibration = 0;
		int16_t gyro_y_calibration = 0;
		int16_t gyro_z_calibration = 0;

		 RCC->APB2ENR |= (1<<2) | (1<<3) | (1<<0); // GPIOA CLOCK ENABLED
		 //RCC->APB2ENR |= (1<<3); // GPIOB CLOCK ENABLED
		// RCC->APB2ENR |= (1<<9); // ADC INTERFACE 1 CLOCK ENABLED
		 RCC->APB1ENR |= (1<<21) | (1<<1) | (1<<0) | (1<<3) | (1<<2); // TIMER 2 & 3  ENABLE
		 //RCC->APB1ENR |= (1<<1); // TIMER 3 ENABLE

		 GPIOB->CRH = 0x88828242; // LED FOR CLIBRATION OF imu B12
		 GPIOB->CRL = 0xFF388838; // B6 B7 FOR IMU I2C
		 GPIOA->CRL = 0xBBBBBBB0; // A1 A2 A3 A6 FOR MOTOR PWM
		 GPIOA->CRH = 0x22222222; // LED FOR CLIBRATION OF MOTOR A9
          GPIOB->ODR &=~(1<<8);
		 //GPIOB->ODR |= (3<<11);
		 //GPIOB->ODR |= (1<<5);
		 //GPIOB->ODR |= (3<<0);
		 init_i2c();
		 imu_awak();
		 //delay(100);
		 signal_lpfilter();
		 //delay(1);

		 acc_z_calibration = imu_cal(63,2048.0);
		 gyro_x_calibration = imu_cal(67,16.4);
		 gyro_y_calibration = imu_cal(69,16.4);
		 gyro_z_calibration = imu_cal(71,16.4);
		 GPIOB->ODR |= (1<<12); // CALIBRATION OF IMU DONE

		 TIM2->CCER |= (0x3333); // Enable capture/compare1 output enable channel 1 & 2 & 3 & 4
		 //TIM2->CCER |= (1<<5); // Capture/Compare 1 output polarity ACTIVE HIGH
		 TIM2->CCMR1 |= (6<<12); // enable pwm mode 1 in channel 2 & channel 1
		 TIM2->CCMR1 |= (6<<4); // enable pwm mode 1 in channel 2 & channel 1
		 TIM2->CCMR1 |= (1<<11) | (1<<3); // CCR1 IS UPDATED ONLY IN UPDATE EVENT preload
		 TIM2->CCMR2 |= (6<<12); // enable pwm mode 1 in channel 3 & channel 4
		 TIM2->CCMR2 |= (6<<4); // enable pwm mode 1 in channel 3 & channel 4
		 TIM2->CCMR2 |= (1<<11) | (1<<3); // CCR1 IS UPDATED ONLY IN UPDATE EVENT preload
		 //TIM2->CR1 |= (2<<5); // center aligned mode 2
		 TIM2->CR1 |= (1<<7); // Auto-reload preload enable

		 TIM3->CCER |= (0x3); // Enable capture/compare1 output enable channel 1 & 2 & 3 & 4
		 //TIM2->CCER |= (1<<5); // Capture/Compare 1 output polarity ACTIVE HIGH
//		 TIM3->CCMR1 |= (6<<12); // enable pwm mode 1 in channel 2 & channel 1
		 TIM3->CCMR1 |= (6<<4); // enable pwm mode 1 in channel 2 & channel 1
		 TIM3->CCMR1 |= (1<<3); // CCR1 IS UPDATED ONLY IN UPDATE EVENT preload
		 TIM3->CR1 |= (1<<7); // Auto-reload preload enable


		 TIM2->ARR = 39999;
		 TIM2->PSC = 3;
		 //TIM2->CCR2 = 0;
		 TIM2->EGR = 1; // UG = 1 Re-initialize the counter and generates an update of the registers
		 TIM2->CR1 |= (1<<0);

		 TIM3->ARR = 39999;
		 TIM3->PSC = 3;
		 //TIM2->CCR2 = 0;
		 TIM3->EGR = 1; // UG = 1 Re-initialize the counter and generates an update of the registers
		 TIM3->CR1 |= (1<<0);


		 //delay(10999,9999); // delay 8 sec
		 /*TIM2->CCR2 = 2667;
		 delay(7999,999); // delay 1 sec

		 TIM2->CCR2 = 0;*/
		 TIM4->PSC = 72 - 1;  										// 72 MHz / 72 = 1 MHz (1 microsecond per tick)
		 TIM4->ARR = 1;  											// Auto-reload register value
		 TIM4->CNT = 0;
		 TIM4->CR1 = TIM_CR1_CEN;
		 while( !( ( GPIOB->IDR & (1<<0) ) == 1 ) ){
			//TIM2->CCR1 = 5332;
			TIM2->CCR2 = 5332;
			TIM2->CCR3 = 5332;
			TIM2->CCR4 = 5332;
			TIM3->CCR1 = 5332 ;

		 }
		  GPIOA->ODR |= (1<<9);

		  Trig_low; 												// turn off trig
		  delayuS(10); 											// wait 10 us
		  Trig_high;  											// turn on trig
		  delayuS(10); 											// wait 10 us
		  Trig_low; 												// turn off trig
		  duration = read_echo(1000000); 							// measure the time of echo pin
		  distance = duration* 0.342 ;							// distance = duration / 2 * SOUND_SPEED
		  delay(10);

		  xTaskCreate(task_motor_1,"motor1",128,NULL,2,NULL);
		  xTaskCreate(task_motor_2,"motor2",128,NULL,2,NULL);
		  xTaskCreate(task_motor_3,"motor3",128,NULL,2,NULL);
		  xTaskCreate(task_motor_4,"motor4",128,NULL,2,NULL);
		  xTaskCreate(task_imu,"imu",128,NULL,1,NULL);
		  //xTaskCreate(task_ultrasonic, "ultrasonic", 128, NULL, 1, NULL);

		  vTaskStartScheduler();

		  while(1);
}


void init_i2c(){
	I2C1->CR1 &= ~(1<<0);
	I2C1->CR2 |= (8<<0);
	I2C1->CCR = (40<<0);
	I2C1->CCR &= ~(1<<15);
	I2C1->TRISE |= (9<<0);
	I2C1->CR1 |= (1<<0);
	//I2C1->CR1 |= (1<<10); // ENABLING ACK BIT
}

void imu_awak(){
	while( (I2C1->SR2 & (1<<1)) != 0);

	//I2C1->CR1 |= (1<<10); // enabling ACK bit
	I2C1->CR1 |= (1<<8); // START I2C BIT
	while( (I2C1->SR1 & (1<<0)) == 0); // CHECKING SB BIT FOR START



	I2C1->DR = 0b11010000; // SENDING ADRRESS
	while( ( ( I2C1->SR1 & (1<<1) ) == 0 ) ); // WAITING FOR ADDR = 1
	int temp = I2C1->SR1 | I2C1->SR2; // READING SR1 AND SR2 TO CLEAR ADDR BIT

	if( (I2C1->SR1 & (1<<9)) != 0){ // checking if arbitration lost
		return;
	}


	while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
	I2C1->DR = 0x6B; // SENDING AWAKE REGISTER

	while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
	I2C1->DR = 0x00; // WRITING 0x00 TO AWAKE REGISTER

	while( (I2C1->SR1 & (1<<2) ) == 0); // waiting for BTF

	I2C1->CR1 |= (1<<9); // stop

}

void signal_lpfilter(){

	int temp;
	I2C1->CR1 |= (1<<8); // START I2C BIT
	while( (I2C1->SR1 & (1<<0)) == 0); // CHECKING SB BIT FOR START


	I2C1->DR = 0b11010000; // SENDING ADRRESS
	while( ( ( I2C1->SR1 & (1<<1) ) == 0 ) ); // WAITING FOR ADDR = 1
	temp = I2C1->SR1 | I2C1->SR2; // READING SR1 AND SR2 TO CLEAR ADDR BIT

	if( (I2C1->SR1 & (1<<9)) != 0){ // checking if arbitration lost
		return;
	}


	while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
	I2C1->DR = 0x1A; // SENDING ConfigurationCONFIG REGISTER

	while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
	I2C1->DR = 0x05; // WRITING 0x05 TO ConfigurationCONFIG REGISTER bandwidth = 5

	while( (I2C1->SR1 & (1<<2) ) == 0); // waiting for BTF

	I2C1->CR1 |= (1<<9); // stop

	// increasing scale of gyroscope = +-2000 sensitivy = 16.4

	I2C1->CR1 |= (1<<8); // START I2C BIT
		while( (I2C1->SR1 & (1<<0)) == 0); // CHECKING SB BIT FOR START


		I2C1->DR = 0b11010000; // SENDING ADRRESS
		while( ( ( I2C1->SR1 & (1<<1) ) == 0 ) ); // WAITING FOR ADDR = 1
		temp = I2C1->SR1 | I2C1->SR2; // READING SR1 AND SR2 TO CLEAR ADDR BIT

		if( (I2C1->SR1 & (1<<9)) != 0){ // checking if arbitration lost
			return;
		}


		while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
		I2C1->DR = 0x1B; // SENDING ConfigurationCONFIG REGISTER

		while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
		I2C1->DR = 0x18; // WRITING 0x18 TO Configure gyroscope scale = +-2000

		while( (I2C1->SR1 & (1<<2) ) == 0); // waiting for BTF

		I2C1->CR1 |= (1<<9); // stop

		//setting accelerometer to full scale = 16g sensitivy = 2048

		I2C1->CR1 |= (1<<8); // START I2C BIT
			while( (I2C1->SR1 & (1<<0)) == 0); // CHECKING SB BIT FOR START


			I2C1->DR = 0b11010000; // SENDING ADRRESS
			while( ( ( I2C1->SR1 & (1<<1) ) == 0 ) ); // WAITING FOR ADDR = 1
			temp = I2C1->SR1 | I2C1->SR2; // READING SR1 AND SR2 TO CLEAR ADDR BIT

			if( (I2C1->SR1 & (1<<9)) != 0){ // checking if arbitration lost
				return;
			}


			while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
			I2C1->DR = 0x1C; // SENDING Configuration REGISTER

			while( ( I2C1->SR1 & (1<<7) ) == 0); // waiting for ack
			I2C1->DR = 0x18; // WRITING 0x18 TO Configure accelerometer scale = +-16g

			while( (I2C1->SR1 & (1<<2) ) == 0); // waiting for BTF

			I2C1->CR1 |= (1<<9); // stop

}

float read_imu(int sensor,float sensitivity){
	I2C1->CR1 |= (1<<10); // enabling ACK bit
	I2C1->CR1 |= (1<<8); // START I2C BIT
	while( ( I2C1->SR1 & (1<<0) ) == 0); // CHECKING SB BIT FOR START

	I2C1->DR = 0b11010000;
	//delay(10000);
	while( ( ( I2C1->SR1 & (1<<1) ) == 0 ) ); // WAITING FOR ADDR = 1
	int temp = I2C1->SR1 | I2C1->SR2; // READING SR1 AND SR2 TO CLEAR ADDR BIT

	while( (I2C1->SR1 & (1<<7) ) == 0);// waiting for ack
	I2C1->DR = sensor; // sending address of first register of desired reading

	while( (I2C1->SR1 & (1<<2) ) == 0);// waiting for BTF

	I2C1->CR1 |= (1<<10); // enabling ACK bit
	I2C1->CR1 |= (1<<8); // repeated start
	while( (I2C1->SR1 & (1<<0)) == 0); // WAITING FOR SB BIT FOR START

	I2C1->DR = 0b11010001;
	while( ( I2C1->SR1 & (1<<1) ) == 0 ); // WAITING FOR ADDR = 1
	int x = I2C1->SR1 | I2C1->SR2;

	while( (I2C1->SR1 & (1<<6) ) == 0); // WAIT FOR RXNE = 1
	int16_t value = ( I2C1->DR<<8 ); // READING FIRST MSB 8 byte of DATA SENT
	//I2C1->CR1 |= (1<<10); // SENDING ACK
	I2C1->CR1 &= ~(1<<10); // clearing ACK bit

	I2C1->CR1 |= (1<<9); // stop bit

	while( (I2C1->SR1 & (1<<6) ) == 0); // WAIT FOR RXNE = 1
	value |= ((I2C1->DR)<<0);
	float f_value = (value/sensitivity);

	return f_value;
}

int16_t imu_cal(int sensor,float sensitivity){
	int16_t c_value = 0;
	int16_t temp;
	for(int i = 0; i < 2000; i++){
		if(sensor == 59 | sensor == 61 | sensor == 63){
			temp = (read_imu(sensor,sensitivity))*9.8;
		}else{
			temp = (read_imu(sensor,sensitivity));
		}
		c_value += temp;
		delay(1);
	}
	c_value /= 2000;
	return c_value;
}

int16_t gyro_angle(float state, float input){
	//int16_t angle = read_imu();
	int16_t angle = state + ( input*0.008 );
	return angle;
}

void kalman_filter(float state, float uncertainty, float input_gyro, float measurment_acc){
	state = state + input_gyro*0.007; // gyro reading angle
	uncertainty = uncertainty + (0.008*0.008 + 10*10); // uncertainty of the gyro reading angle
	float gain = uncertainty/(uncertainty + 3*3); // ratio of uncertainty of gyro to error of accelerometer angle
	state = state + gain*(measurment_acc - state); // new better predict of angle
	uncertainty = (1-gain)*uncertainty; // uncertainty of the better angle

	kalman_values[0] = state;
	kalman_values[1] = uncertainty;
}
void kalman_filter_roll(float state, float uncertainty, float input_gyro, float measurment_acc){
	state = state + input_gyro*0.007; // gyro reading angle
	uncertainty = uncertainty + (0.008*0.008 + 10*10); // uncertainty of the gyro reading angle
	float gain = uncertainty/(uncertainty + 3*3); // ratio of uncertainty of gyro to error of accelerometer angle
	state = state + gain*(measurment_acc - state); // new better predict of angle
	uncertainty = (1-gain)*uncertainty; // uncertainty of the better angle

	kalman_values_roll[0] = state;
	kalman_values_roll[1] = uncertainty;
}

void delay(int d){
	TIM4->PSC = 999-1; // prescaller value
	TIM4->ARR = d; // time is d milliseconds
	TIM4->SR = 0; // clear SR bit
	TIM4->CR1 = 1; // START OF UP COUNTING
	while( (TIM4->SR & 1 ) == 0 );
	TIM4->CR1 = 0; // STOP OF COUNTING
}


//void delaymS(uint32_t ms) //delay for certain amount in milliseconds
//{
//    SysTick->LOAD = 72000 - 1; // 72 MHz / 1000 = 72,000 ticks per millisecond
//    SysTick->VAL = 0;
//    SysTick->CTRL = 0x5;
//    for (int i = 0; i < ms; i++)
//    {
//        while (!(SysTick->CTRL & 0x10000)){}
//    }
//    SysTick->CTRL = 0;
//}

//void delayuS(uint32_t us) //delay for certain amount in microseconds
//{
//	us=us/8;
//    for (int i = 0; i < us; i++)
//    {
////        TIM4->CNT = 0;  // Reset the counter
////        while (!(TIM4->SR & TIM_SR_UIF)) {}  // Wait for update event
////        TIM4->SR &= ~TIM_SR_UIF;  // Clear update event flag
//
//    }
//}
void delayuS(uint32_t us) //delay for certain amount in microseconds
{

	for (int i = 0; i < us; i++)
	    {
	        TIM4->CNT = 0;  // Reset the counter
	        while (!(TIM4->SR & TIM_SR_UIF)) {}  // Wait for update event
	        TIM4->SR &= ~TIM_SR_UIF;  // Clear update event flag
	    }
}

uint32_t read_echo(uint32_t timeout)
{
	duration = 0 ;
	    while (!((GPIOB->IDR) & (1<<9)))
	    {
	        duration++;
	        delayuS(1);
	        if (duration > timeout)
	        {
	            return 0;
	        }
	    }
		duration = 0 ;
	    while ((GPIOB->IDR & (1<<9)))
	    {
	        duration++;
	        delayuS(1);
	        if (duration > timeout)
	        {
	            return 0;
	        }
	    }
	    return duration;
}
