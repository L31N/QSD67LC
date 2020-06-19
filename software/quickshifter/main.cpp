
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include "uart.h"

#define UART_BAUD_RATE      9600

const unsigned int STATUS_LED_BLUE  = 0;
const unsigned int STATUS_LED_GREEN = 1;
const unsigned int STATUS_LED_RED   = 2;

void init();
bool button();
bool shiftSensor();
void setLed(unsigned char led, bool value);
void setIgnition(bool value);
void printShifttime(unsigned char shifttime);

void printRPM(unsigned int rpm);

volatile unsigned int rpm_frequency;
volatile unsigned int rpm_slope_count;

ISR(TIMER0_OVF_vect)
{
    // do the RPM frequency calculation here and update volatile variable
    rpm_frequency = rpm_slope_count / 60 / (F_CPU / 1024 / 255);
    rpm_slope_count = 0;
}

ISR(INT1_vect) {
    rpm_slope_count++;
}

int main () {

    init();

    bool setupMode = false;
    unsigned int shifttime_ms = 85;
    unsigned int disabletime_ms = 1000;

    while (true) {
        /** SETUP MODE **** */
        if (button() && !setupMode) {   // enter setup mode
            setupMode = true;
            setLed(STATUS_LED_GREEN, true);

            printShifttime(shifttime_ms);

            while(true) {
                if (button()) printShifttime(shifttime_ms);
            }
        }

        // print RPM
        printRPM(rpm_frequency);
        _delay_ms(100);


        /** SHIFT MODE **** */
//        if (shiftSensor()) {
//            setLed(STATUS_LED_RED, true);
//            setLed(STATUS_LED_GREEN, true);
//            setIgnition(false);
//            for (int i = 0; i < shifttime_ms; i++) _delay_ms(1);
//            setIgnition(true);
//            setLed(STATUS_LED_GREEN, false);
//            for (int i = 0; i < disabletime_ms; i++) _delay_ms(1);
//            setLed(STATUS_LED_RED, false);
//        }
    }
}

void init() {
    DDRC |= 0x07;       // LEDs as output
    DDRC &= ~(1 << 5);  // button as input

    DDRD &= ~(1 << 2);  // shift sensor as input

    DDRD |= (1 << 7);   // IGN_CTL as output

    DDRB |= (1 << 0);   // HC05_RESET as output

    // uart ---
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
    sei();

    // interrupts for QS+ and RPM
    //GICR |= (1 << 6);     // QS+ (INT0)
    GICR |= (1 << 7);       // RPM (INT1)

    // lowlevel rises interrupt on QS+
    //MCUCR &= ~(1 << 0);
    //MCUCR &= ~(1 << 1);

    // rising edge rises interrupt on RPM
    MCUCR |= (1 << 2);
    MCUCR |= (1 << 3);


    // initialize frequency
    rpm_slope_count = 0;
    rpm_frequency = 0;

    // timer settings for RPM detection
    TCCR0 |= (1 << CS02)|(1 << CS00);   // prescaler: 1024
    TCNT0 = 0;                          // initialize counter

    TIMSK |= (1 << TOIE0);              // enable timer overflow interrupt for timer0


    // set ignition on for default
    PORTD |= (1 << 7);

    // status led flash
    PORTC &= ~0x07;                                 // all leds on
    for (int i = 0; i < 100; i++) _delay_ms(10);    // delay of 1 sec
    PORTC |= 0x06;                                  // just blue led on

    PORTB &= ~(1 << 0);                              // HC-05 Reset off
    PORTB |= (1 << 0);
}

bool button() {
    return PINC & (1 << 5);
}

bool shiftSensor() {
    //return (button() || !(PIND & (1 << 2)));

    //return button();
    return (PIND & (1 << 2));
}

void setLed(unsigned char led, bool value) {
    if (value == true) {
        PORTC &= ~(1 << led);
    }
    else {
        PORTC |= (1 << led);
    }
}

void setIgnition(bool value) {
    if (value == true) {
        PORTD |= (1 << 7);
    }
    else {
        PORTD &= ~(1 << 7);
    }
}

void printShifttime(unsigned char shifttime) {
    uart_puts("\r#################################\n");
    uart_puts("\r# Welcome to QSD67LC Setup Mode #\n");
    uart_puts("\r#################################\n\n");

    char strbuf[4];
    itoa(shifttime, strbuf, 10);


    uart_puts("\rSHIFT-TIME (ms): [");
    uart_puts(strbuf);
    uart_puts("]");
}

void printRPM(unsigned int rpm) {
    char strbuf[6];
    itoa(rpm, strbuf, 10);

    uart_puts("\rRPM [");
    uart_puts(strbuf);
    uart_puts("]\n");
}

