#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int UART_init(){
	UART_flag = 0;
	UART_buf_len = 0;
	UART_buf = (unsigned char*) malloc(sizeof(unsigned char) * UART_MAX_SIZE);
	buf= (unsigned char*) malloc(sizeof(unsigned char) * UART_MAX_SIZE);
	return 0;
}

int UART_reset(){
	UART_flag = 0;
	UART_buf_len = 0;
	return 0;
}

int UART_out(char out){
	slip_arch_writeb(out);
	return 1;
}

int UART_input_callback(unsigned char c){
	UART_out(UART_buf_len);

	UART_buf[UART_buf_len] = c;
	UART_buf_len ++;

	if (UART_buf_len >= 10){
		UART_flag = 1;
	}
	return 0;
}

int UART_read(){
	memcpy(buf, UART_buf, UART_buf_len);
	buf_len = UART_buf_len;
	UART_reset();
	return buf_len;
}

int UART_print_buf(){
	int i = 0;
	for(i=0; i<UART_buf_len;i++){
		UART_out(UART_buf[i]);
	}
	return UART_buf_len;
}
