/*
 * Serial.cpp
 *
 *  Created on: Aug 24, 2020
 *      Author: Macbook
 */

#include "Serial.h"

Serial::Serial( UART_HandleTypeDef * uartx, Mode_print mode_p )
{
    // TODO Auto-generated constructor stub
    mode = mode_p;
    uart = uartx;
}

void Serial::receive( uint8_t * data, uint16_t timeout,
        Mode_timeout_t modeTimeout )
{
    uint8_t *bufer = data;
    uint32_t lastTime = HAL_GetTick();

    *bufer = (uint8_t) uart->Instance->DR;
    bufer++;

    if ( modeTimeout == TIMEOUT )
    {
        while ( ( HAL_GetTick() - lastTime ) < timeout )
        {
            if ( __HAL_UART_GET_FLAG( uart, UART_FLAG_RXNE ) )
            {
                *bufer = (uint8_t) ( uart->Instance->DR & (uint8_t) 0x00FF );
                bufer++;
            }
        }
    }
    else if(modeTimeout == INTER_BYTE_TIMEOUT)
    {
        if(timeout == 1) timeout++;

        while ( ( HAL_GetTick() - lastTime ) < timeout )
        {
            if ( __HAL_UART_GET_FLAG( uart, UART_FLAG_RXNE ) )
            {
                *bufer = (uint8_t) uart->Instance->DR ;
                bufer++;
                lastTime = HAL_GetTick();
            }
        }
    }
}

void Serial::write( uint8_t * text )
{
    uint8_t i = 0;

    if ( mode == PRINT_UART )
    {
        HAL_UART_Transmit( uart, text, strlen( (char*) text ), 1000 );
    }
    else
    {
        for ( i = 0; i < strlen( (char*) text ); i++ )
        {
            ITM_SendChar( text[i] );
        }
    }
}

