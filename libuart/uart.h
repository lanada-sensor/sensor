#ifndef UART_H
#define UART_H

#define UART_INIT 1

#define UART_MAX_SIZE 100

/* uar variable */

extern unsigned char* UART_buf;
extern unsigned char* buf;
extern int UART_buf_len;
extern int UART_flag;
extern int buf_len;
/* uart functions */

int UART_init();
int UART_reset();
int UART_out(char out);
int UART_input_callback(unsigned char c);
int UART_read();
int UART_print_buf();

#endif /* NTP_H */
