
#include <avr/io.h>
#include <avr/interrupt.h>

#include "suart.h"


#define nop() __asm volatile ("nop")

static volatile uint16_t outframe;

static volatile uint16_t inframe;
static volatile uint8_t inbits, received;

static volatile uint8_t indata;


void suart_init(unsigned int baudrate) {

    uint8_t tifr = 0;
    uint8_t sreg = SREG;
    cli();

    // Mode #4 für Timer1
    // und volle MCU clock
    // IC Noise Cancel
    // IC on Falling Edge
    TCCR1A = 0;
    //TCCR1B = (1 << WGM12) | (1 << CS10) | (0 << ICES1) | (1 << ICNC1);
    TCCR1B = (1 << WGM12) | (1 << CS10) | (0 << ICES1) | (1 << ICNC1);

    // OutputCompare für gewünschte Timer1 Frequenz
    OCR1A = (uint16_t) ((uint32_t) F_CPU/baudrate);

    SUART_RXD_DDR  &= ~(1 << SUART_RXD_BIT);
    SUART_RXD_PORT |=  (1 << SUART_RXD_BIT);
    TIMSK |= (1 << TICIE1);
    tifr  |= (1 << ICF1) | (1 << OCF1B);

    tifr |= (1 << OCF1A);
    SUART_TXD_PORT &= (1 << SUART_TXD_BIT);
    SUART_TXD_DDR  |= (1 << SUART_TXD_BIT);
    outframe = 0;

    TIFR = tifr;

    SREG = sreg;
}



ISR (TIMER1_CAPT_vect)
{
    uint16_t icr1  = ICR1;
    uint16_t ocr1a = OCR1A;

    // Eine halbe Bitzeit zu ICR1 addieren (modulo OCR1A) und nach OCR1B
    uint16_t ocr1b = icr1 + ocr1a/2;
    if (ocr1b >= ocr1a)
        ocr1b -= ocr1a;
    OCR1B = ocr1b;

    TIFR = (1 << OCF1B);
    TIMSK = (TIMSK & ~(1 << TICIE1)) | (1 << OCIE1B);
    inframe = 0;
    inbits = 0;
}

ISR (TIMER1_COMPB_vect)
{
    uint16_t data = inframe >> 1;

    if (SUART_RXD_PIN & (1 << SUART_RXD_BIT))
        data |= (1 << 9);

    uint8_t bits = inbits+1;

    if (10 == bits) {
        if ((data & 1) == 0)
            if (data >= (1 << 9))
            {
                PORTC &= ~(1 << 1);
                indata = data >> 1;
                received = 1;
            }

        TIMSK = (TIMSK & ~(1 << OCIE1B)) | (1 << TICIE1);
        TIFR = (1 << ICF1);
    }
    else
    {
        inbits = bits;
        inframe = data;
    }
}

char suart_getc()
{
    while (!received)   {}
    received = 0;

    return (int) indata;
}

char suart_getc_nonblocking()
{
    if (received)
    {
        received = 0;
        return (int) indata;
    }

    return -1;
}



void suart_putc (const char c)
{
    do
    {
        sei(); nop(); cli(); // yield();
    } while (outframe);

    // frame = *.P.7.6.5.4.3.2.1.0.S   S=Start(0), P=Stop(1), *=Endemarke(1)
    outframe = (3 << 9) | (((uint8_t) c) << 1);

    TIMSK |= (1 << OCIE1A);
    TIFR   = (1 << OCF1A);

    sei();
}



ISR (TIMER1_COMPA_vect)
{
    uint16_t data = ~outframe;

    if (data & 1)      SUART_TXD_PORT |=  (1 << SUART_TXD_BIT);
    else               SUART_TXD_PORT &= ~(1 << SUART_TXD_BIT);

    if (1 == ~data)
    {
        TIMSK &= ~(1 << OCIE1A);
    }

    outframe = (~data) >> 1;
}
