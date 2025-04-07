#ifndef SERIAL_PORT_STUFF_H
#define SERIAL_PORT_STUFF_H
#include "main.h"
#include <string.h>
#include <stdbool.h>


UART_HandleTypeDef huart2;


volatile uint8_t dataReceived = 0;
uint8_t rxBuffer[100];  // Buffer for received data
uint8_t rxIndex = 0;

bool isTyped(const char* text) {
	return strcasecmp((char*)rxBuffer, text) == 0;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)  // Check if it's the correct UART
  {
      if (rxBuffer[rxIndex] == '\b' || rxBuffer[rxIndex] == 127)  // Backspace (ASCII 0x08 or DEL 127)
      {
          if (rxIndex > 0) // Ensure there's something to delete
          {
              rxIndex--;  // Move index back
              rxBuffer[rxIndex] = '\0'; // Remove character

              // Send backspace sequence to erase character on terminal
              char backspaceSeq[] = "\b \b";
              HAL_UART_Transmit(&huart2, (uint8_t*)backspaceSeq, strlen(backspaceSeq), HAL_MAX_DELAY);
          }
      }
      else if (rxBuffer[rxIndex] == '\n' || rxBuffer[rxIndex] == '\r')  // Enter key
      {
          rxBuffer[rxIndex] = '\0';  // Null-terminate the string

          // Move to a new line for better readability
          char newLine[] = "\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)newLine, strlen(newLine), HAL_MAX_DELAY);

          // Print received text
          char debugMessage[30];
          sprintf(debugMessage, "You typed: %s\r\n", rxBuffer);
          HAL_UART_Transmit(&huart2, (uint8_t*)debugMessage, strlen(debugMessage), HAL_MAX_DELAY);

          // Check if "LED" was typed
          if (strcasecmp((char*)rxBuffer, "LED") == 0)
          {
              HAL_GPIO_TogglePin(GPIOA, LED_Pin_Pin); // Toggle LED
              char ledMessage[] = "LED Toggled!\r\n";
              HAL_UART_Transmit(&huart2, (uint8_t*)ledMessage, strlen(ledMessage), HAL_MAX_DELAY);
          }

          rxIndex = 0;  // Reset buffer index
      }
      else  // Normal character input
      {
          if (rxIndex < sizeof(rxBuffer) - 1)  // Prevent buffer overflow
          {
              // Echo received character
              HAL_UART_Transmit(&huart2, &rxBuffer[rxIndex], 1, HAL_MAX_DELAY);
              rxIndex++;  // Move buffer index forward
          }
      }

      // Restart UART reception
      HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
  }


}

#include "main.h"
#include <stdio.h>
#include <string.h>

// Print a text string over UART
void printText(const char* text) {
    HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

// Print a text string with a newline
void printlnText(const char* text) {
    printText(text);
    printText("\r\n");  // Add newline
}

// Print an integer over UART
void printInt(int number) {
    char buffer[20];  // Buffer to store the converted number
    sprintf(buffer, "%d", number);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Print an integer with a newline
void printlnInt(int number) {
    printInt(number);
    printText("\r\n");  // Add newline
}

// Print a float over UART
void printFloat(float number, int decimalPlaces) {
    char buffer[30];  // Buffer to store the converted float
    sprintf(buffer, "%.*f", decimalPlaces, number);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Print a float with a newline
void printlnFloat(float number, int decimalPlaces) {
    printFloat(number, decimalPlaces);
    printText("\r\n");  // Add newline
}


#endif
