#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "ADCstuff.h"
#include "serialPortStuff.h"

const uint8_t gain_array[16][5] = {
		//SWR_IN1, SWR_IN2, SWR_IN3, SWR_IN4, overall gain
		{0,0,0,0,2.92},
		{0,0,0,1,6.96},
		{0,0,1,0,13.23},
		{0,0,1,1,17.28},
		{0,1,0,0,28.97},
		{0,1,0,1,33.02},
		{0,1,1,0,39.29},
		{0,1,1,1,43.33},
		{1,0,0,0,86.82},
		{1,0,0,1,90.86},
		{1,0,1,0,97.13},
		{1,0,1,1,101.17},
		{1,1,0,0,112.87},
		{1,1,0,1,116.91},
		{1,1,1,0,123.19},
		{1,1,1,1,127.23}
};

void SW1_gain_select(void){
HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)

};
}
