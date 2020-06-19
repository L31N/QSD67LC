
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#include <avr/eeprom.h>

#include <avr/interrupt.h>
#include "uart.h"

#define UART_BAUD_RATE      9600

const unsigned int STATUS_LED_ENABLED   = 0; // blue
const unsigned int STATUS_LED_SETUPMODE = 1; // green
const unsigned int STATUS_LED_SPARK     = 2; // red

const unsigned char CMD_NULL            = 0x00;
const unsigned char CMD_DISABLE         = 0x01;
const unsigned char CMD_ENDABLE         = 0x02;
const unsigned char CMD_SET_SHIFT       = 0x04;
const unsigned char CMD_GET_SHIFT       = 0x08;
const unsigned char CMD_GET_RPM         = 0x10;
const unsigned char CMD_PRINT_RPM       = 0x20;
const unsigned char CMD_SET_THRESHOLD   = 0x40;
const unsigned char CMD_GET_THRESHOLD   = 0x80;

const unsigned char CMD_SETUP           = 0xF0;
const unsigned char CMD_RESET           = 0xFF;


const unsigned char EEPROM_ADDRESS_ENABLED          = 0x01;
const unsigned char EEPROM_ADDRESS_LOWER_THRESHOLD  = 0x02;

const unsigned char EEPROM_ADDRESS_SHIFT_TIME[14] {
    0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
};

// RPM_FACTOR = CPU_CLOCK * 60 s/m / (PRESCALER * 2^(timer_width)) / 2 (unsure where this 2 comes from)
const double RPM_FACTOR = F_CPU * 60 / (256 * pow(2,16)) / 2;


void init();
bool button();
bool shiftSensor();
void setLed(unsigned char led, bool value);
void toggleLed(unsigned char led);
void setIgnition(bool value);

unsigned char rpmToIndex(unsigned int rpm);

void printSetupMode();
void printSingleShiftTime(unsigned char* shifttime, unsigned char index);
void printShifttime(unsigned char* shifttime);
void printRPM(unsigned int rpm);
void printError(char* errorstr);
void printThreshold(unsigned char threshold);

bool readSetupModeEnable();
bool readCommand(unsigned char& cmd, unsigned char& value0, unsigned char& value1);

volatile unsigned int rpm_frequency;
volatile unsigned int rpm_slope_count;



bool fenabled = false;                  // will be overwritten with eeprom value
bool fsetupMode = true;

unsigned char lower_threshold = 3;      // don't disable ignition below 3000rpm -- stored in eeprom
unsigned int disabletime_ms = 1000;     // time to disable for sensor debouncing

unsigned char shifttime_ms[14] = {0};     // this is stored in eeprom

ISR(TIMER1_OVF_vect)
{
    // do the RPM frequency calculation here and update volatile variable
    rpm_frequency = rpm_slope_count * RPM_FACTOR;
    rpm_slope_count = 0;
}

ISR(INT1_vect) {
    rpm_slope_count++;
    //toggleLed(STATUS_LED_SPARK);
}

int main () {

    init();

    while (true) {
        /** SETUP MODE **** */
        if (readSetupModeEnable()) {
            fsetupMode = true;
            setLed(STATUS_LED_SETUPMODE, true);
            printSetupMode();

            while(fsetupMode) {
                unsigned char cmd;
                unsigned char cmdvalue;
                unsigned char cmdvalue1;
                if (readCommand(cmd, cmdvalue, cmdvalue1)) {
                    switch (cmd) {
                        case CMD_DISABLE:
                            uart_puts("\rquickshifter disabled ...\n");
                            fenabled = false;
                            eeprom_write_byte((uint8_t*)EEPROM_ADDRESS_ENABLED, fenabled);
                            break;

                        case CMD_ENDABLE:
                            uart_puts("\rquickshifter enabled ...\n");
                            fenabled = true;
                            eeprom_write_byte((uint8_t*)EEPROM_ADDRESS_ENABLED, fenabled);
                            break;

                        case CMD_SET_SHIFT:
                            // CMD INDEX VALUE
                            shifttime_ms[cmdvalue] = cmdvalue1;
                            eeprom_write_byte((uint8_t*)(EEPROM_ADDRESS_SHIFT_TIME[cmdvalue]), cmdvalue1);

                            uart_puts("\r changed shifting time:\n");
                            printSingleShiftTime(shifttime_ms, cmdvalue);
                            break;

                        case CMD_GET_SHIFT:
                            printShifttime(shifttime_ms);
                            break;

                        case CMD_GET_RPM:
                            printRPM(rpm_frequency);
                            break;

                        case CMD_PRINT_RPM:
                            while(!button()) {
                                printRPM(rpm_frequency);
                                _delay_ms(1000);
                            }
                            break;

                        case CMD_SET_THRESHOLD:
                            lower_threshold = cmdvalue;
                            eeprom_write_byte((uint8_t*)EEPROM_ADDRESS_LOWER_THRESHOLD, cmdvalue);
                            uart_puts("\r changed lower threshold\n");
                            printThreshold(lower_threshold);
                            break;

                        case CMD_GET_THRESHOLD:
                            printThreshold(lower_threshold);
                            break;


                        case CMD_RESET:
                            uart_puts("\rleaving setup mode");
                            fsetupMode = false;
                            break;
                    }
                } else {
                    // could not read uart command
                    printError((char*)"Error while receiving command");
                }
            }
            setLed(STATUS_LED_SETUPMODE, false);
        }


        /** SHIFT MODE **** */
        if (fenabled && !shiftSensor()) {
            setLed(STATUS_LED_ENABLED, false);
            setIgnition(false);
            for (unsigned int i = 0; i < shifttime_ms[rpmToIndex(rpm_frequency)]; i++) _delay_ms(1);
            setIgnition(true);
            for (unsigned int i = 0; i < disabletime_ms; i++) _delay_ms(1);
            setLed(STATUS_LED_ENABLED, true);
        }

        setLed(STATUS_LED_ENABLED, fenabled);
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



    /// read settings from eeprom
    fenabled = eeprom_read_byte((uint8_t*)EEPROM_ADDRESS_ENABLED);

    for (unsigned int i = 0; i < 14; i++) {
        shifttime_ms[i] = eeprom_read_byte((uint8_t*)(EEPROM_ADDRESS_SHIFT_TIME[i]));
    }

    /// disable all LEDs
    setLed(STATUS_LED_ENABLED, false);
    setLed(STATUS_LED_SETUPMODE, false);
    setLed(STATUS_LED_SPARK, false);

    /*_delay_ms(1000);
    setLed(STATUS_LED_ENABLED, true);
    _delay_ms(1000);
    setLed(STATUS_LED_ENABLED, false);
    setLed(STATUS_LED_SETUPMODE, true);
    _delay_ms(1000);
    setLed(STATUS_LED_SETUPMODE, false);
    setLed(STATUS_LED_SPARK, true);*/
}

bool button() {
    return PINC & (1 << 5);
}

bool shiftSensor() {
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

unsigned char rpmToIndex(unsigned int rpm) {
    if (rpm >= 13500) return 13;
    else if (rpm >= 12500) return 12;
    else if (rpm >= 11500) return 11;
    else if (rpm >= 10500) return 10;
    else if (rpm >= 9500) return 9;
    else if (rpm >= 8500) return 8;
    else if (rpm >= 7500) return 7;
    else if (rpm >= 6500) return 6;
    else if (rpm >= 5500) return 5;
    else if (rpm >= 4500) return 4;
    else if (rpm >= 3500) return 3;
    else if (rpm >= 2500) return 2;
    else if (rpm >= 1500) return 1;
    else if (rpm >= 500) return 0;
    else return 0;
}

void printSetupMode() {
    uart_puts("\r#################################\n");
    uart_puts("\r# Welcome to QSD67LC Setup Mode #\n");
    uart_puts("\r#################################\n\n");

    uart_puts("\rplease enter command:\n");
}

void printSingleShiftTime(unsigned char* shifttime, unsigned char index) {

    char strbuf_rpm[5];
    char strbuf_st[5];
    itoa((index+1), strbuf_rpm, 10);
    itoa(shifttime[index], strbuf_st, 10);

    uart_puts("\rSHIFT-TIME (ms): [");
    uart_puts(strbuf_rpm);
    uart_puts("000rpm / ");
    uart_puts(strbuf_st);
    uart_puts("ms]\n");

    uart_puts("\n");
}

void printShifttime(unsigned char* shifttime) {

    uart_puts("\rSHIFT-TIME (ms): \n");
    for (unsigned int i = 0; i < 14; i++) {

        char strbuf_rpm[5];
        char strbuf_st[5];
        itoa((i+1), strbuf_rpm, 10);
        itoa(shifttime[i], strbuf_st, 10);

        uart_puts("\r[");
        uart_puts(strbuf_rpm);
        uart_puts("000rpm / ");
        uart_puts(strbuf_st);
        uart_puts("ms]\n");
        _delay_ms(100);   // delay to avoid flooding the uart
    }
    uart_puts("\n");
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

void printThreshold(unsigned char threshold) {
    char strbuf[4];
    itoa(threshold, strbuf, 10);

    uart_puts("\rThreshold [");
    uart_puts(strbuf);
    uart_puts("]\n");
}



bool readSetupModeEnable() {
    unsigned int cmdbytes = uart_getc();
    if (cmdbytes == CMD_SETUP) return true;
    else return false;
}

bool readCommand(unsigned char& cmd, unsigned char& value0, unsigned char& value1) {
    /// always receive three byte (cmd and cmdvalue)
    _delay_ms(50);  // wait for all bytes are there
    unsigned int cmdbytes = uart_getc();
    unsigned int valuebytes0 = uart_getc();
    unsigned int valuebytes1 = uart_getc();

    if ((cmdbytes & 0xFF00) == 0) {       // received new cmd
        if ((valuebytes0 & 0xFF00) == 0 && (valuebytes1 & 0xFF00) == 0) { // received new value bytes
            cmd = (cmdbytes & 0xFF);
            value0 = (valuebytes0 & 0xFF);
            value1 = (valuebytes1 & 0xFF);
            return true;
        } else {
            return false;
        }
    } else if ((cmdbytes & 0xFF00) == UART_NO_DATA) {
        cmd = CMD_NULL;
        value0 = 0;
        value1 = 0;
        return true;
    } else {
        return false;
    }
}

