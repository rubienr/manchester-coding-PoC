/**
 * simple Mancester coding implementation
 *
 * sends cyclic the byte stored in data with a clock defined in Counter1Settings
 * a pause is appended after each transmission
 *
 * @author raoul rubien
 * 01/2016
 */
#include <Arduino.h>

/**
 * PORTD2 - pin 19 - compare B interrupt handler duration
 * PORTD1 - pin 20 - compare A interrupt handler duration
 * PORTD0 - pin 21 - signal output pin - manchester code
 * PORTA0 - pin 22 - clock out pin
 * PORTA1 - pin 23 - data out bit pin
 */


typedef struct CounterSettings {
    const uint16_t top; // the max value the counter 1 is counting to - OCR1A
    const uint16_t center; // top / 2
    const uint8_t prescaler; // OCCR prescaler
    const uint8_t transmissionPauseDelay; // amount of package clocks to pause after byte is sent
};

const CounterSettings Counter1Settings = { //
        .top = 500, //
        .center = Counter1Settings.top / 2, //
        // const uint8_t TimerClk = (0 << CS12) | (0 << CS11) | (0 << CS10), // disconnected
        .prescaler = (0 << CS12) | (0 << CS11) | (1 << CS10), // clk/1
        // const uint8_t prescaler = (0 << CS12) | (1 << CS11) | (0 << CS10), // clk/8
        // const uint8_t prescaler = (0 << CS12) | (1 << CS11) | (1 << CS10), // clk/64
        // const uint8_t prescaler = (1 << CS12) | (0 << CS11) | (0 << CS10), // clk/256
        // const uint8_t prescaler = (1 << CS12) | (0 << CS11) | (1 << CS10), // clk/1024
        .transmissionPauseDelay = 10, //
};


uint8_t bitMask = 0;
uint8_t data = 0b11100101;
uint8_t transmissionPauseCounter = Counter1Settings.transmissionPauseDelay;

// trigger clock
ISR(TIMER1_COMPA_vect) {
    PORTD |= (1 << PORTD1);
    TCNT1 = 0;

    if (bitMask == 0) {
        PORTA &= ~(1 << PORTA0);
        PORTA &= ~(1 << PORTA1);
        PORTD |= (1 << PORTD0);
        transmissionPauseCounter = Counter1Settings.transmissionPauseDelay;
    } else {
        // clock out high
        PORTA |= (1 << PORTA0);

        // bit value out
        uint8_t bitValue = ((bitMask & data));
        if (bitValue) {
            PORTA |= (1 << PORTA1);
        } else {
            PORTA &= ~(1 << PORTA1);
        }

        // generated signal out
        if (!(bitMask & data)) {
            PORTD |= (1 << PORTD0);
        } else {
            PORTD &= ~(1 << PORTD0);
        }
    }
    PORTD &= ~(1 << PORTD1);
}

// trigger transmission
ISR(TIMER1_COMPB_vect) {
    PORTD |= (1 << PORTD2);

    if (transmissionPauseCounter == 0) {
        if (bitMask != 0) {
            // clock out low
            PORTA &= ~(1 << PORTA0);

            // bit value out
            uint8_t bitValue = ((bitMask & data));
            if (bitMask & data) {
                PORTA |= (1 << PORTA1);
            } else {
                PORTA &= ~(1 << PORTA1);
            }

            // generated signal out
            if ((bitMask & data)) {
                PORTD |= (1 << PORTD0);
            } else {
                PORTD &= ~(1 << PORTD0);
            }
            bitMask <<= 1;
        }
    } else {
        // pause transmission
        if (--transmissionPauseCounter == 0) {
            bitMask = 1;
            TCNT1 = Counter1Settings.center + 1;
            TIMSK1 |= (1 << OCIE1A);
        } else { // start transmission
            TCNT1 = 0;
            TIMSK1 &= ~(1 << OCIE1A);
        }

    }
    PORTD &= ~(1 << PORTD2);
}


void setup() {
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(20, OUTPUT);
    pinMode(19, OUTPUT);


    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << COM1C1) | (0 << COM1C0);
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << CS12) | (0 << CS11) | (0 << CS10);

    TCCR1A |= (0 << WGM11) | (0 << WGM10);
    TCCR1B |= (0 << WGM13) | (0 << WGM12);

    TIMSK1 = (0 << ICIE1) | (0 << OCIE1C) | (1 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);
    TIMSK0 = 0;

    OCR1B = Counter1Settings.center;
    OCR1A = Counter1Settings.top;
    sei();
    TCCR1B |= Counter1Settings.prescaler;
}


void loop() {
    while (1);
}
