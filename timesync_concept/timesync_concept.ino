/*
 * simple Manchester decoding implementation
 * @author raoul rubien
 *
 * receives manchester coding
 * synchronizes and adopts the reception clock to te reception received coding
 *
 * 01/2016
 */
#include <Arduino.h>
#include <avr/interrupt.h>
#include<avr/cpufunc.h>

typedef struct InitialCounterSettings {
    const uint16_t top; // the max value the counter 1 is counting to - OCR1A
    const uint16_t receptionDelta; // allowed reception deviation of center / top
    const uint16_t leftOfTop; // minimum reception counter at top
    const uint16_t rightOfTop; // maximum reception counter at top
    const uint16_t center; //
    const uint16_t leftOfCenter; // minimum reception counter at center
    const uint16_t rightOfCenter; // maximum reception counter at center
    const uint16_t syncCheck; // when to check if bit was received or transmission terminated
    const uint8_t prescaler;
};

const InitialCounterSettings Counter1Settings = { //
        .top = 500, //
        .receptionDelta = (Counter1Settings.top / 4) - 10, //
        .leftOfTop = Counter1Settings.top - Counter1Settings.receptionDelta, //
        .rightOfTop = Counter1Settings.receptionDelta,  //
        .center = Counter1Settings.top / 2, //
        .leftOfCenter = Counter1Settings.center - Counter1Settings.receptionDelta, //
        .rightOfCenter = Counter1Settings.center + Counter1Settings.receptionDelta, //
        .syncCheck = Counter1Settings.center / 2, //
        // const uint8_t prescaler = (0 << CS12) | (0 << CS11) | (0 << CS10), // disconnected
        .prescaler = (0 << CS12) | (0 << CS11) | (1 << CS10), // clk/1
        // const uint8_t prescaler = (0 << CS12) | (1 << CS11) | (0 << CS10), // clk/8
        //const uint8_t prescaler = (0 << CS12) | (1 << CS11) | (1 << CS10), // clk/64
        // const uint8_t prescaler = (1 << CS12) | (0 << CS11) | (0 << CS10), // clk/256
        // const uint8_t prescaler = (1 << CS12) | (0 << CS11) | (1 << CS10), // clk/1024
};


uint8_t isReceiving = 0;

/*
 * PORTH4 - pin 7 - timer counter 1 output signal pin
 * PORTH3 - pin 6 - is-package-synced. output signal pin
 * PINK5 - reception input pin
 * PORTD0 - pin 21 - TIMER1_COMPA_vect interrupt handling duration - green/3
 * PORTD1 - pin 20 - PCINT2_vect interrupt handling duration - blue/2
 * PORTD2 -  pin 19 - received data bit output pin -
 * PORTD3 - pin 18 - TIMER1_COMPB_vect interrupt handling duration - violet/1
 * PORTB7 - pin 13 - pin where to listen for input interrupt/data
 *
 * */


// on timer end interrupt: reset timer
ISR(TIMER1_COMPA_vect) {
    PORTD |= (1 << PORTD0);

    TCNT1 = 0;
    PORTH ^= (1 << PORTH4);

    PORTD &= ~(1 << PORTD0);
}

// on sync check interrupt: update status on whether a bit was received or elpsend bit -> transmission end
ISR(TIMER1_COMPB_vect) {
    PORTD |= (1 << PORTD3);

    if (isReceiving == 0) {
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
    } else {
        --isReceiving;
    }

    if (isReceiving == 0) {
        PORTH &= ~(1 << PORTH3); // unset sync bit
        PORTD &= ~(1 << PORTD2); // unset data out
    } else {
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
        _NOP();_NOP();
    }

    PORTD &= ~(1 << PORTD3);
}

// on received signal interrupt: classify, handle reception and synchronize/adopt timer
ISR(PCINT2_vect) {
    uint8_t isRisingEdge = (PINK & (1 << PINK5)); // edge on PCINT21 - pin 5
    PORTD |= (1 << PORTD1);

    uint16_t captureCounter = TCNT1;

    // 1st signal of a package:
    // + synchronize counter
    if (isReceiving == 0) { // 4
        if (isRisingEdge == 0) { // 1
            TCNT1 = 0; // 4
            PORTH |= (1 << PORTH4); // clk bit // 5
            PORTH |= (1 << PORTH3); // set sync bit // 5
            PORTD &= ~(1 << PORTD2); // unset data out // 2
        } else { // 2
            _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
            _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
            _NOP();_NOP(); // 2
        }
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP(); // 8
    }

    else { // receiving signal of a package // 2

        // if signal occurs approx. at 1/2 of a package clock:
        // + the flank direction defines a data bit
        // + approximate new counter value
        if ((Counter1Settings.leftOfCenter <= captureCounter) && // 6
            (captureCounter <= Counter1Settings.rightOfCenter)) // 16
        {
            OCR1A = (Counter1Settings.top + (Counter1Settings.top + 2 * (captureCounter - Counter1Settings.center))) /
                    2; // 19
            uint16_t newB = OCR1A / 4; // 4
            if (OCR1B < captureCounter) { // 2
                OCR1B = newB; // 4
            } else {// 2
                _NOP();_NOP();_NOP();_NOP(); // 4
            } // 2

            if (isRisingEdge == 0) { // 2
                PORTD &= ~(1 << PORTD2); // 4
            } else { // 2
                PORTD |= (1 << PORTD2); // 4
            }
        } // 2
            // if signal occurs approx. at the end/beginning of a package clock:
            // + synchronize counter
        else // 2
        if ((Counter1Settings.leftOfTop <= captureCounter) || // 8
            (captureCounter <= Counter1Settings.rightOfTop)) { // 9
            TCNT1 = 0; // 4
            PORTH ^= (1 << PORTH4); // 6
            OCR1B = OCR1A / 4; // 12
            _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
            _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
            _NOP();
        } // 2
    } // 2

    isReceiving = 2;
    TIFR1 = (1 << OCF1A) | (1 << OCF1B);

    PORTD &= ~(1 << PORTD1);
}


void setup() {

    cli();

    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);

    pinMode(20, OUTPUT);
    digitalWrite(20, LOW);

    pinMode(19, OUTPUT);
    digitalWrite(19, LOW);

    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);

    pinMode(13, INPUT);
    digitalWrite(13, HIGH);

    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);

    pinMode(6, OUTPUT);
    digitalWrite(6, LOW);

    // disconnect output compare port operation
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << COM1C1) | (0 << COM1C0);
    // wave generator mode: normal
    TCCR1B = (0 << WGM13) | (0 << WGM12);
    TCCR1A |= (0 << WGM11) | (0 << WGM10);

    // input compare noise canelling: off
    TCCR1B |= (0 << ICNC1) | (0 << ICES1);
    // counter prescaler: disconnected
    TCCR1B |= (0 << CS12) | (0 << CS11) | (0 << CS10);

    // output compare interrupt enable
    TIMSK1 = (0 << ICIE1) | (0 << OCIE1C) | (1 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1);


    // enable PCINT[23:16] interrupt on port K
    PCICR |= (1 << PCIE2);
    // enable interrupt PCINT21 to contribute
    PCMSK2 |= (1 << PCINT21);


    TCNT1 = 0;
    OCR1A = Counter1Settings.top;
    OCR1B = Counter1Settings.syncCheck;
    sei();
    TCCR1B |= Counter1Settings.prescaler; // enable counting
}


void loop() {
    uint8_t verbose = false;

    if (verbose) {
        Serial.begin(19200);
        Serial.print("[0 tr ");
        Serial.print(Counter1Settings.rightOfTop);
        Serial.print(" cl ");
        Serial.print(Counter1Settings.leftOfCenter);
        Serial.print(" c ");
        Serial.print(Counter1Settings.center);
        Serial.print(" cr ");
        Serial.print(Counter1Settings.rightOfCenter);
        Serial.print(" sc ");
        Serial.print(Counter1Settings.syncCheck);
        Serial.print(" tl ");
        Serial.print(Counter1Settings.leftOfTop);
        Serial.print(" t ");
        Serial.print(Counter1Settings.top);
        Serial.print("] - d ");
        Serial.println(Counter1Settings.receptionDelta);
        while (1) {
            Serial.print("sc ");
            Serial.print(OCR1B);
            Serial.print(" top ");
            Serial.println(Counter1Settings.top);
            delay(1000);
        }
    } else {
        while (1);
    }
}

