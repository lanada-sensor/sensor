#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int UART_out(char out){
	slip_arch_writeb(out);
	return 1;
}

int UART_input_callback(unsigned char c){
	UART_out(c);
	return 0;
}
