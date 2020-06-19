
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include "suart.h"

#define BAUDRATE      10400

#define UART_TX_PIN     PD1
#define UART_TX_PORT    PORTD


const unsigned int STATUS_LED_BLUE  = 0;
const unsigned int STATUS_LED_GREEN = 1;
const unsigned int STATUS_LED_RED   = 2;

void init();
bool button();
bool shiftSensor();
void setLed(unsigned char led, bool value);
void toggleLed(unsigned char led);
void setIgnition(bool value);

void kLineSlowInit();

void showError(unsigned char number);
void showStatus(unsigned char status);

int main() {

    init();

    while(true) {
        while(!button());

        //setLed(STATUS_LED_GREEN, true);
        kLineSlowInit();
        //setLed(STATUS_LED_GREEN, false);

        suart_init(BAUDRATE);
        sei();
        _delay_ms(20);

        setLed(STATUS_LED_BLUE, false);

        char syncrr = suart_getc();
        if (syncrr == 0x55) { // starting init
            setLed(STATUS_LED_BLUE, true);
            char key1 = suart_getc();
            char key2 = suart_getc();
            _delay_ms(100);
            suart_putc(0x42);
            _delay_ms(10);
            suart_putc(key1);
            _delay_ms(20);
            suart_putc(key2);
            _delay_ms(20);

            suart_putc(~key1);
            char ready = suart_getc();
            if (true) {
                setLed(STATUS_LED_BLUE, true);

                if (key1 == 0x08 && key2 == 0x08) { // ISO 9141
                    showStatus(10);
                }
                else if (key2 == 0x8f) {            // ISO 14230-4 KWP
                    showStatus(5);
                }
                else {  // something went wrong
                    showError(15);
                }
            }
            else {  // not ready
                showError(10);
            }
        }
        else {  // init failed
            showError(5);
        }
    }

    return 0;
}

void init() {
    DDRC |= 0x07;       // LEDs as output

    DDRC &= ~(1 << 5);  // button as input

    DDRD &= ~(1 << 2);  // shift sensor as input

    DDRD |= (1 << 7);   // IGN_CTL as output

    //DDRB |= (1 << 0);   // HC05_RESET as output // --> this is used as software uart RX pin

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

void kLineSlowInit() {

    DDRD |= (1 << UART_TX_PIN);

    UART_TX_PORT |= (1 << UART_TX_PIN);
    _delay_ms(200);
    UART_TX_PORT &= ~(1 << UART_TX_PIN);
    _delay_ms(400);
    UART_TX_PORT |= (1 << UART_TX_PIN);
    _delay_ms(400);
    UART_TX_PORT &= ~(1 << UART_TX_PIN);
    _delay_ms(400);
    UART_TX_PORT |= (1 << UART_TX_PIN);
    _delay_ms(400);
    UART_TX_PORT &= ~(1 << UART_TX_PIN);
}

void showError(unsigned char number) {

    char old_port = PORTC;  // make backup of old state

    setLed(STATUS_LED_BLUE, false);
    setLed(STATUS_LED_GREEN, false);

    // flash the number
    for (int i = 0; i < number; i++) {
        setLed(STATUS_LED_RED, true);
        _delay_ms(150);
        setLed(STATUS_LED_RED, false);
        _delay_ms(300);
    }
    setLed(STATUS_LED_RED, true); _delay_ms(1000);
    setLed(STATUS_LED_RED, false);

    // restore old state
    PORTC = old_port;
}

void showStatus(unsigned char status) {

    char old_port = PORTC;  // make backup of old state

    setLed(STATUS_LED_BLUE, false);
    setLed(STATUS_LED_RED, false);

    // flash the number
    for (int i = 0; i < status; i++) {
        setLed(STATUS_LED_GREEN, true);
        _delay_ms(150);
        setLed(STATUS_LED_GREEN, false);
        _delay_ms(300);
    }

    // restore old state
    PORTC = old_port;
}
