#ifndef ADC_STUFF_H
#define ADC_STUFF_H

#include "main.h"
#include <stdio.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

// Define ADC channels and their respective ADC peripherals
typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
    char label[20];  // To label ADC readings in UART output
} ADC_Channel_Config;

// Define each ADC channel with its corresponding ADC instance
ADC_Channel_Config vs_ADC           = { &hadc1, ADC_CHANNEL_1, "Vs_ADC" };
ADC_Channel_Config clean_sine_ADC   = { &hadc3, ADC_CHANNEL_0, "Clean_sine_ADC" };
ADC_Channel_Config clean_cos_ADC    = { &hadc2, ADC_CHANNEL_6, "clean_cos_ADC" };
ADC_Channel_Config v_zg_ADC         = { &hadc2, ADC_CHANNEL_7, "V_zg_ADC" };
ADC_Channel_Config mult_ADC         = { &hadc1, ADC_CHANNEL_14, "mult_ADC" };
ADC_Channel_Config vin_ADC          = { &hadc2, ADC_CHANNEL_15, "Vin_ADC" };
ADC_Channel_Config v_dut_ADC        = { &hadc2, ADC_CHANNEL_9, "V_dut_ADC" };

#define NUM_ADC_CHANNELS 7  // Total ADC channels

ADC_Channel_Config* adcChannels[NUM_ADC_CHANNELS] = {
    &vs_ADC,
    &clean_sine_ADC,
    &clean_cos_ADC,
    &v_zg_ADC,
    &mult_ADC,
    &vin_ADC,
    &v_dut_ADC
};



// Function to read ADC from a given ADC_Channel_Config object
uint32_t readADC(ADC_Channel_Config *adcConfig)
{
    static ADC_HandleTypeDef *prev_hadc = NULL;
    static uint32_t prev_channel = 0xFFFFFFFF;  // Invalid initial value to force first-time initialization

    uint32_t adcValue = 0;
    ADC_ChannelConfTypeDef sConfig = {0};  // ADC configuration struct

    // Only reconfigure if the ADC handle or channel has changed
    if (adcConfig->hadc != prev_hadc || adcConfig->channel != prev_channel)
    {
        sConfig.Channel = adcConfig->channel;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Use longer sampling for better stability

        if (HAL_ADC_ConfigChannel(adcConfig->hadc, &sConfig) != HAL_OK)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)"ADC Config Error\r\n", 18, HAL_MAX_DELAY);
            return 0xFFFFFFFF; // Return an error code
        }

      HAL_Delay(10);  // Allow stabilization after switching channels
    }

    // Start ADC conversion
    if (HAL_ADC_Start(adcConfig->hadc) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"ADC Start Error\r\n", 18, HAL_MAX_DELAY);
        return 0xFFFFFFFF; // Return an error code
    }

    // Poll ADC conversion with a timeout
    if (HAL_ADC_PollForConversion(adcConfig->hadc, 100) == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(adcConfig->hadc);
    }
    else
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"ADC Read Error\r\n", 16, HAL_MAX_DELAY);
        adcValue = 0xFFFFFFFF; // Return an error code
    }

    HAL_ADC_Stop(adcConfig->hadc);

    // Update previous ADC channel and ADC handle to prevent unnecessary reconfiguration
    prev_hadc = adcConfig->hadc;
    prev_channel = adcConfig->channel;

    return adcValue;
}


// Function to read all ADC channels and send values over UART
void Read_All_ADC_Values()
{
    char message[50];
    uint32_t adcValue;

    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\nReading ADC Channels...\r\n", 27, HAL_MAX_DELAY);

    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
        adcValue = readADC(adcChannels[i]); // Use Read_Specific_ADC for each channel

        // Format the message and send it via UART
        sprintf(message, "%s: %lu\r\n", adcChannels[i]->label, adcValue);
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

        HAL_Delay(50);  // Small delay between readings for stability
    }
}


#endif
