#ifndef _SUART_H_
#define _SUART_H_

#define SUART_RXD_DDR   DDRB
#define SUART_RXD_PORT  PORTB
#define SUART_RXD_PIN   PINB
#define SUART_RXD_BIT   PB0

#define SUART_TXD_DDR   DDRD
#define SUART_TXD_PORT  PORTD
#define SUART_TXD_PIN   PIND
#define SUART_TXD_BIT   PD1

void suart_init(unsigned int baudrate);

void suart_putc(char byte);
char suart_getc();

char suart_getc_nonblocking();

#endif _SUART_H_ // _UART_H_
