/*
 * sbus.h
 *
 *  Created on: Nov 26, 2019
 *      Author: Đặng Lâm Tùng
 */
#include <stdint.h>
#include "main.h"
typedef struct{
	uint16_t esc_value1;
	uint16_t esc_value2;
	uint16_t esc_value3;
	uint16_t esc_value4;
	uint16_t crc;
}ESC_value;




ESC_value sbus_decode(uint8_t data[7]){
	ESC_value value;
	value.esc_value1 = (data[0] << 3) | ((data[1] & 0b11100000)>>5);
	value.esc_value2 = ((data[1] & 0b00011111)<<6)|((data[2] & 0b11111100)>>2);
	value.esc_value3 = (((data[2] &0b00000011)<<9)|(data[3]<<1))|((data[4] & 0b10000000)>>7);
	value.esc_value4 = ((data[4] & 0b01111111)<<4)|(data[5])>>4;
    value.crc = data[6];
	return value;
}
bool check_CRC(ESC_value value){
    uint16_t check = value.esc_value1 + value.esc_value2 + value.esc_value3 + value.esc_value4;
    if(check % 37 != value.crc)
    	return false;
    return true;
}

bool CRC_thurst(ESC_value value){

    if((value.esc_value1 == value.esc_value2) && (value.esc_value3 == value.esc_value4) && (value.esc_value1 == value.esc_value4))
    	return true;
    return false;
}

void init_ESC(){
	  HAL_TIM_Base_Start(&htim4);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);

        for(int i = 0; i < 5; i++){
        	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        	HAL_Delay(50);
        	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        	HAL_Delay(50);
        	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        	HAL_Delay(50);
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	    HAL_Delay(1000);
}

void set_ESC(ESC_value esc){
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,esc.esc_value1);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, esc.esc_value2);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,esc.esc_value3);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,esc.esc_value4);
}
