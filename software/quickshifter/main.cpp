
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include "uart.h"

#define UART_BAUD_RATE      9600

const unsigned int STATUS_LED_BLUE  = 0;
const unsigned int STATUS_LED_GREEN = 1;
const unsigned int STATUS_LED_RED   = 2;

const unsigned char CMD_DISABLE         = 0x01;
const unsigned char CMD_ENDABLE         = 0x02;
const unsigned char CMD_SET_SHIFT       = 0x04;
const unsigned char CMD_GET_SHIFT       = 0x08;
const unsigned char CMD_GET_RPM         = 0x10;
const unsigned char CMD_PRINT_RPM       = 0x20;

const unsigned char CMD_RESET           = 0xFF;


// RPM_FACTOR = CPU_CLOCK * 60 s/m / (PRESCALER * 2^(timer_width)) / 2 (unsure where this 2 comes from)
const double RPM_FACTOR = F_CPU * 60 / (256 * pow(2,16)) / 2;


void init();
bool button();
bool shiftSensor();
void setLed(unsigned char led, bool value);
void toggleLed(unsigned char led);
void setIgnition(bool value);

void printSetupMode();
void printShifttime(unsigned char shifttime);
void printRPM(unsigned int rpm);
void printError(char* errorstr);

bool readCommand(unsigned char& cmd, unsigned char& value);

volatile unsigned int rpm_frequency;
volatile unsigned int rpm_slope_count;

ISR(TIMER1_OVF_vect)
{
    toggleLed(STATUS_LED_GREEN);
    // do the RPM frequency calculation here and update volatile variable
    rpm_frequency = rpm_slope_count * RPM_FACTOR;
    rpm_slope_count = 0;
}

ISR(INT1_vect) {
    rpm_slope_count++;
    toggleLed(STATUS_LED_RED);
}

int main () {

    init();

    bool fenabled = true;
    bool fsetupMode = false;

    unsigned int shifttime_ms = 85;
    unsigned int disabletime_ms = 1000;

    while (true) {
        /** SETUP MODE **** */
        if (button() && !fsetupMode) {   // enter setup mode
            fsetupMode = true;
            setLed(STATUS_LED_BLUE, false);

            _delay_ms(1000);    // debunce button

            printSetupMode();

            unsigned char cmd;
            unsigned char cmdvalue;
            if (readCommand(cmd, cmdvalue)) {
                if (cmd == CMD_DISABLE) {

                }
            } else {
                // could not read uart command
                printError("Error while receiving command");
            }
        }

        /** SHIFT MODE **** */
        if (shiftSensor()) {
            setLed(STATUS_LED_RED, true);
            setLed(STATUS_LED_GREEN, true);
            setIgnition(false);
            for (unsigned int i = 0; i < shifttime_ms; i++) _delay_ms(1);
            setIgnition(true);
            setLed(STATUS_LED_GREEN, false);
            for (unsigned int i = 0; i < disabletime_ms; i++) _delay_ms(1);
            setLed(STATUS_LED_RED, false);
        }
    }
}

void init() {
    DDRC |= 0x07;       // LEDs as output
    DDRC &= ~(1 << 5);  // button as input

    DDRD &= ~(1 << 2);  // shift sensor as input
    //DDRD &= ~(1 << 3);  // RPM interrupt as input

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

    // timer settings for RPM detection (TCNT1 => 16 bit timer)
    TCCR1B |= (1 << CS12);                  // prescaler: 256
    //TCCR1B |= (1 << CS11)|(1 << CS10);      // prescaler: 64
    //TCCR1B |= (1 << CS12)|(1 << CS10);    // prescaler: 1024
    TCNT1 = 0;                              // initialize counter

    TIMSK |= (1 << TOIE1);                  // enable timer overflow interrupt for timer1


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

void toggleLed(unsigned char led) {
    PORTC ^= (1 << led);
}

void setIgnition(bool value) {
    if (value == true) {
        PORTD |= (1 << 7);
    }
    else {
        PORTD &= ~(1 << 7);
    }
}

void printSetupMode() {
    uart_puts("\r#################################\n");
    uart_puts("\r# Welcome to QSD67LC Setup Mode #\n");
    uart_puts("\r#################################\n\n");

    uart_puts("\rplease enter command: ");
}

void printShifttime(unsigned char shifttime) {
    char strbuf[4];
    itoa(shifttime, strbuf, 10);


    uart_puts("\rSHIFT-TIME (ms): [");
    uart_puts(strbuf);
    uart_puts("]");
}

void printRPM(unsigned int rpm) {
    char strbuf[16];
    itoa(rpm, strbuf, 10);

    uart_puts("\rRPM [");
    uart_puts(strbuf);
    uart_puts("]\n");
}

void printError(char* errorStr) {
    uart_puts("Error: ");
    uart_puts(errorStr);
    uart_puts("\n");
}

