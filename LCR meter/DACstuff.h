#ifndef DAC_STUFF_H
#define  DAC_STUFF_H

#include "main.h"

void Write_To_DAC(DAC_HandleTypeDef *hdac, uint32_t channel, uint32_t value)
{
    // Ensure the value does not exceed the 12-bit DAC resolution (0 - 4095)
    if (value > 4095) value = 4095;

    // Set the DAC output value
    if (HAL_DAC_SetValue(hdac, channel, DAC_ALIGN_12B_R, value) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"DAC Write Error\r\n", 17, HAL_MAX_DELAY);
        return;
    }

    // Start the DAC (if not already started)
    if (HAL_DAC_Start(hdac, channel) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"DAC Start Error\r\n", 18, HAL_MAX_DELAY);
        return;
    }
}

void writeToSineDAC(int val) {
	Write_To_DAC(&hdac, DAC_CHANNEL_1, val);  // Set DAC output to mid-scale (1.65V on a 3.3V system)
}

void writeToCosDAC(int val) {
	Write_To_DAC(&hdac, DAC_CHANNEL_2, val);  // Set DAC output to mid-scale (1.65V on a 3.3V system)
}

#endif
