#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>
#include <util/delay.h>
#include "lcd.h"

/*
   The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 6
 * LCD D5 pin to digital pin 7
 * LCD D6 pin to digital pin 4
 * LCD D7 pin to digital pin 5
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 *
 * Encoder : 6,7
 */

#define temp A0
#define CLK 3
#define DT 2
#define SW 6

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */    
}

int uart_putchar(char c, FILE *stream) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

char uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar,
                                     NULL,
                                     _FDEV_SETUP_WRITE
                                     );

#define TIMER_FREQ_HZ   1

void timer1_init(void){
    // Init Pin PB0 Output, set to 0
    DDRB |= (1<<DDB0);
    PORTB &= ~(1<<PORTB0);

    cli();
    // initialize timer1
    //noInterrupts();           // disable all interrupts
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Set FastPWM with OCR1A on TOP, CLEAR on Compare + Set on BOTTOM
    // Prescaler 256
    // Here 64 !
    TCCR1A = _BV(COM1A1)
            | _BV(COM1B1)
            | _BV(WGM10)
            | _BV(WGM11);

    TCCR1B = _BV(WGM12)
            | _BV(WGM13)
            | _BV(CS11)
            | _BV(CS10);

    OCR1B = 0;//156;//10*3125/100;
    OCR1A = 3125;
    sei();             // enable all interrupts
}

int main(void)
{
//	char c = ' ';

    uint8_t test[]   = "12222";
    uart_init();
	
	//sei();
	

    stdout = &uart_output;

    //timer1_init();
    lcd_init_4d();
    
    lcd_write_string_4d(test);
    printf("Debut\r\n");
	for (;;) {
    }	
    return 0; /* never reached */
}
