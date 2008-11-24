/*
USART
*/

#ifndef _USART_H_
#define _USART_H_

#include <stdio.h>
#include "defines.h"
#define BAUD 19200
#include <util/setbaud.h>
void usart_init(void);
static int usart_putchar(char c, FILE *stream);
int usart_getchar(FILE *stream);

#endif