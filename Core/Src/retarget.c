#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

#if defined ( __GNUC__ )
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
