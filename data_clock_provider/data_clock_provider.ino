#include <Arduino.h>
#include <avr/fuse.h>

const uint8_t redLed = 3;
const uint8_t yellowLed = 5;
const uint8_t outputSignalPin = 21;
const uint8_t outputCPin = 22;
const uint8_t outputDPin = 23;

const uint8_t joystickXPin = 0;
const uint8_t joystickYPin = 1;

const uint8_t joystick2XPin = 4;
const uint8_t joystick2YPin = 5;


// timer counter 1 prescaler
// const uint8_t timerClk = (0 << CS12) | (0 << CS11) | (0 << CS10); // disconnected
const uint8_t timerClk = (0 << CS12) | (0 << CS11) | (1 << CS10); // clk/1
// const uint8_t timerClk = (0 << CS12) | (1 << CS11) | (0 << CS10); // clk/8
// const uint8_t timerClk = (0 << CS12) | (1 << CS11) | (1 << CS10); // clk/64
// const uint8_t timerClk = (1 << CS12) | (0 << CS11) | (0 << CS10); // clk/256
// const uint8_t timerClk = (1 << CS12) | (0 << CS11) | (1 << CS10); // clk/1024

uint16_t compareA = 4000; // min 4000 @ prescaler=1, internal osc F_CPU=8000000;
const uint16_t maxPkgClkDrift = (compareA / 4) * 0.9;
uint16_t compareB = compareA / 2;

uint8_t bitCounter = 0;
uint8_t data = 0b11100101;
uint8_t isPackageClockHi = false;

uint8_t idx = 0;

typedef enum GeneratorModes {
    MODE_1BYTE, MODE_CLOCK, MODE_CALIBRATE, MODE_NUMBER_OF_TYPES
};
const char *GeneratorModeNames[] = {"MODE_1BYTE", "MODE_CLOCK", "MODE_CALIBRATE"};
GeneratorModes generatorMode = MODE_1BYTE;

ISR(TIMER1_COMPC_vect) {
    TCCR1B = 0;
    TCNT1H = 0;
    TCNT1L = 0;
    TCCR1B |= timerClk;
    digitalWrite(outputCPin, !digitalRead(outputCPin));
}

ISR(TIMER1_COMPA_vect) {
    TCCR1B = 0;
    TCNT1H = 0;
    TCNT1L = 0;

    OCR1AH = (0xFF00 & compareA) >> 8;
    OCR1AL = 0x00FF & compareA;

    compareB = compareA / 2;
    OCR1BH = (0xFF00 & compareB) >> 8;
    OCR1BL = 0x00FF & compareB;

    if (generatorMode == MODE_1BYTE) {

        isPackageClockHi = (isPackageClockHi == true) ? false : true;
        digitalWrite(yellowLed, isPackageClockHi);

        if (bitCounter != 0) {
            uint8_t bitValue = ((bitCounter & data) != 0);
            uint8_t output = (isPackageClockHi == false) ^(bitValue == false);
            /*
                    Serial.print(idx);
                    Serial.print(" T bv ");
                    Serial.print(bitValue);
                    Serial.print(" c ");
                    Serial.print(isPackageClockHi);
                    Serial.print(" out  ");
                    Serial.println(output);
                    */
            digitalWrite(outputCPin, isPackageClockHi);
            digitalWrite(outputDPin, bitValue);
            digitalWrite(outputSignalPin, output);
            digitalWrite(redLed, HIGH);
            TCCR1B |= timerClk;
        } else {
            digitalWrite(outputCPin, LOW);
            digitalWrite(outputDPin, LOW);
            digitalWrite(outputSignalPin, HIGH);
            digitalWrite(redLed, LOW);
            TCCR1B = 0;
        }
    } else {
        digitalWrite(outputSignalPin, !digitalRead(outputSignalPin));
        TCCR1B |= timerClk;
    }

}


ISR(TIMER1_COMPB_vect) {

    TCCR1B = 0;
    digitalWrite(yellowLed, !digitalRead(yellowLed));

    if (generatorMode == MODE_1BYTE) {
        isPackageClockHi = (isPackageClockHi == true) ? false : true;
        digitalWrite(yellowLed, isPackageClockHi);

        if (bitCounter != 0) {
            uint8_t bitValue = ((bitCounter & data) != 0);
            uint8_t output = (isPackageClockHi == false) ^(bitValue == false);
            /*
                    Serial.print(idx);
                    Serial.print(" C bv ");
                    Serial.print(bitValue);
                    Serial.print(" c ");
                    Serial.print(isPackageClockHi);
                    Serial.print(" out  ");
                    Serial.println(output);
            */

            // data out  <= clk-state xor data-bit
            digitalWrite(outputCPin, isPackageClockHi);
            digitalWrite(outputDPin, bitValue);
            digitalWrite(outputSignalPin, output);
            bitCounter <<= 1;
            digitalWrite(redLed, HIGH);
        } else {
            digitalWrite(outputCPin, LOW);
            digitalWrite(outputDPin, LOW);
            digitalWrite(outputSignalPin, HIGH);
            digitalWrite(redLed, LOW);
        }
        idx++;
    } else {
        digitalWrite(outputSignalPin, !digitalRead(outputSignalPin));
    }
    delayMicroseconds(20);
    TCCR1B |= timerClk;
}


void setup() {

    //    // enable clock prescaler change (page 48)
    //    CLKPR = (1 << CLKPCE) | (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);
    //    // set prescaler factor to 1 (page 49)
    //    CLKPR = (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);
    //
    //    //internal osc user calibration 45, 48, 359
    //    OSCCAL = (1 << CAL7) | (0 << CAL6) | (0 << CAL5) | (0 << CAL4) | (0 << CAL3) | (0 << CAL2) | (0 << CAL1) |
    //             (0 << CAL0);

    //Serial.begin(115200);
    Serial.begin(19200);
    Serial.println("\n\nreset");
    Serial.print("top ");
    Serial.println(compareA);


    pinMode(outputSignalPin, OUTPUT);
    pinMode(redLed, OUTPUT);
    pinMode(yellowLed, OUTPUT);
    pinMode(outputCPin, OUTPUT);
    pinMode(outputDPin, OUTPUT);
    digitalWrite(outputSignalPin, HIGH);


    TCNT1H = 0;
    TCNT1L = 0;

    OCR1AH = (0xFF00 & compareA) >> 8;
    OCR1AL = 0x00FF & compareA;

    TCCR1A = 0;
    TCCR1A |= (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << COM1C1) | (0 << COM1C0) |
              (0 << WGM11) | (0 << WGM10);

    TCCR1B = 0;
    TCCR1B |= (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);

    TIMSK1 = 0;
    TIMSK1 |= (0 << ICIE1) | (0 << OCIE1C) | (1 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1);

    sei();
    TCCR1B |= timerClk;
}


void loop() {

    uint16_t joyStickLowThreshold = 300, joyStickUpperThreshold = 700;
    uint16_t oneTimeOffset = 200;
    uint16_t minCompareA = compareA - maxPkgClkDrift, maxCompareA = compareA + maxPkgClkDrift;
    uint16_t currentOffset = 0;
    uint8_t printCounter = 0;


    while (1) {
        /*
        // read offset and scale transmission clock
        int x = analogRead(joystickXPin);
        if (x >= joyStickUpperThreshold) {
            currentOffset = oneTimeOffset;
        } else if (x <= joyStickLowThreshold) {
            currentOffset = -oneTimeOffset;
        }

        if (currentOffset != 0) {
            uint16_t newA = compareA + currentOffset;
            newA = (newA < minCompareA) ? minCompareA : newA;
            newA = (newA > maxCompareA) ? maxCompareA : newA;

            TCCR1B |= 0;
            compareA = newA;
            currentOffset = 0;
            TCCR1B |= timerClk;
        }

        // read OSC calibraton request
        uint8_t oscCalCaptureCounter = 0;
        if ((oscCalCaptureCounter++ % 127) == 0) {

            // read low bits offset request
            uint16_t oscCalOffsetRequest = analogRead(joystick2XPin);
            uint8_t currentOscLowOffset = (0x7F & OSCCAL);

            if (oscCalOffsetRequest > joyStickUpperThreshold) {
                currentOscLowOffset = (currentOscLowOffset >= 0x7E) ? 0x7F : (currentOscLowOffset + 1);
                OSCCAL = (0x80 & OSCCAL) | (0x7F & currentOscLowOffset);
            } else if (oscCalOffsetRequest < joyStickLowThreshold) {
                currentOscLowOffset = (currentOscLowOffset <= 0) ? 0 : (currentOscLowOffset - 1);
                OSCCAL = (0x80 & OSCCAL) | (0x7F & currentOscLowOffset);
            }

            // read high bit offset request
            oscCalOffsetRequest = analogRead(joystick2YPin);

            if (oscCalOffsetRequest > joyStickUpperThreshold) {
                OSCCAL = 0x80 | (0x7F & OSCCAL);
            } else if (oscCalOffsetRequest < joyStickLowThreshold) {
                OSCCAL = (0x7F & OSCCAL);
            }
        }
        */

        /*
                // status msg
                printCounter++;
                if ((printCounter % 100) == 0) {
                    Serial.print("top ");
                    Serial.print(compareA);
                    Serial.print(" center ");
                    Serial.print(compareB);
                    Serial.print(" mode ");
                    Serial.print(GeneratorModeNames[(uint8_t )generatorMode]);
                    Serial.print(" osc cal high ");
                    Serial.print((0x80 & OSCCAL) >> 7);
                    Serial.print(" osc cal low ");
                    Serial.println(0x7F & OSCCAL);
                }

                // read mode switcher
                int y = analogRead(joystickYPin);

                if (y >= joyStickUpperThreshold) {
                    generatorMode = (GeneratorModes)(((uint8_t) (generatorMode + 1)) % ((uint8_t) MODE_NUMBER_OF_TYPES));
                } else if (y <= joyStickLowThreshold) {
                    generatorMode = (GeneratorModes)(((uint8_t) (generatorMode + 1)) % ((uint8_t) MODE_NUMBER_OF_TYPES));
                }
         */

        // rectify for next transmission
        if ((generatorMode == MODE_1BYTE) && (bitCounter == 0)) {
            TCCR1B = 0;
            TCNT1H = (0xFF00 & (compareB + 1)) >> 8;
            TCNT1L = 0x00FF & (compareB + 1);

            isPackageClockHi = false;
            digitalWrite(outputCPin, LOW);
            digitalWrite(outputDPin, LOW);
            digitalWrite(outputSignalPin, HIGH);

            idx = 0;
            bitCounter = 1;
            TIMSK1 = (0 << ICIE1) | (0 << OCIE1C) | (1 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1);

        } else if (generatorMode == MODE_CALIBRATE) { // generate signal for OSCCAL experiment
            uint16_t delay = 1000;
            OCR1CH = (0xFF00 & delay) >> 8;
            OCR1CL = 0x00FF & delay;
            TIMSK1 = (0 << ICIE1) | (1 << OCIE1C) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);
        } else if (generatorMode == MODE_CLOCK) { // send bare signal
            TIMSK1 = (0 << ICIE1) | (0 << OCIE1C) | (1 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1);
        }

        delay(12);
        TCCR1B |= timerClk;
    }

}
