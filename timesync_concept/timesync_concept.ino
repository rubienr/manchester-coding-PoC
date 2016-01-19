#include <Arduino.h>
#include <avr/interrupt.h>

const uint8_t redLed = 3;
const uint8_t yellowLed = 5;

const uint8_t leftMotor = 9;
const uint8_t rightMotor = 10;

const uint8_t compTopOutPin = 7;
//const uint8_t compCenterOutPin = 6;
const uint8_t syncOutPin = 6;

const uint8_t joystickXPin = 0;
const uint8_t joystickYPin = 1;

// pin where to listen for input interrupt/data
const uint8_t inputSignalPin = 13;

const uint8_t topInaccurateOutPin = 21;

// initial approx. data clock
const uint16_t pkgApproxClkCounter = 4000; // min 4000 @ prescaler=1, internal osc F_CPU=8000000;
// top cannot drift below/above pkgApproxClkCounter +/- maxPkgClkDrift
const uint16_t maxPkgClkDrift = (pkgApproxClkCounter / 4) * 0.9;
// top cannot drift below minCompareA
const uint16_t minCompareA = pkgApproxClkCounter - maxPkgClkDrift;
// top cannot drift above maxCompareA
const uint16_t maxCompareA = pkgApproxClkCounter + maxPkgClkDrift;

// top of timer-counter
uint16_t compareA = pkgApproxClkCounter;
// center of timer counter
uint16_t compareB = compareA / 2;
// allowed +/- offset from center when signal is captured
uint16_t centerDelta = (compareA - compareB) / 2;
// allowed +/- offset from top when signal is captured
uint16_t topDelta = centerDelta - 1;

// captures counter on input interrupt
uint16_t captureCounter = 0;

// timer counter 1 prescaler
// const uint8_t timerClk = (0 << CS12) | (0 << CS11) | (0 << CS10); // disconnected
const uint8_t timerClk = (0 << CS12) | (0 << CS11) | (1 << CS10); // clk/1
// const uint8_t timerClk = (0 << CS12) | (1 << CS11) | (0 << CS10); // clk/8
//const uint8_t timerClk = (0 << CS12) | (1 << CS11) | (1 << CS10); // clk/64
// const uint8_t timerClk = (1 << CS12) | (0 << CS11) | (0 << CS10); // clk/256
// const uint8_t timerClk = (1 << CS12) | (0 << CS11) | (1 << CS10); // clk/1024

// max package length in clocks
const uint8_t maxPackageClocks = 8;

// clocks since current package receiving has started
uint16_t packageClock = maxPackageClocks;

// if packageIdleClock > maxPackageIdleClock the package is assumed to has ended
uint8_t packageIdleClock = 0xFF;
const uint8_t maxPackageIdleClocks = 1;

// to filter bouncing signals if they happen twice at the same reception window: center/top
uint8_t centerWindowHandledAtCounter = 0xFF;
uint8_t topWindowHandledAtCounter = 0xFF;


// TODO tov occurs before 1st compare int is triggered
ISR(TIMER1_OVF_vect) {
    digitalWrite(redLed, HIGH);
}


ISR(TIMER1_COMPA_vect) {
    TCCR1B = 0;

    TCNT1H = 0;
    TCNT1L = 0;

    OCR1AH = (0xFF00 & compareA) >> 8;
    OCR1AL = 0x00FF & compareA;
    TCCR1B |= timerClk;

    /*
    compareB = compareA / 2;
    OCR1BH = (0xFF00 & compareB) >> 8;
    OCR1BL = 0x00FF & compareB;
    */
    packageClock = (packageClock < 0xFFFF) ? packageClock + 1 : 0xFFFF;
    packageIdleClock = (packageIdleClock < 0xFF) ? packageIdleClock + 1 : 0xFF;

    /*
    // too many clocks occurred without receiving bits -> package end
    if (packageIdleClock > maxPackageIdleClocks) {
        digitalWrite(syncOutPin, LOW);
        digitalWrite(redLed, LOW);
    }
    */

    // reset de-bouncing marks
    centerWindowHandledAtCounter = 0xFF;
    topWindowHandledAtCounter = 0xFF;

    //    digitalWrite(yellowLed, !digitalRead(yellowLed));
    digitalWrite(compTopOutPin, !digitalRead(compTopOutPin));

}


//ISR(TIMER1_COMPB_vect) {
//    digitalWrite(yellowLed, HIGH);
//    // digitalWrite(compCenterOutPin, !digitalRead(compCenterOutPin));
//}


float smoothFactor1 = 0.9; // center
float smoothFactor2 = 0.9; // top: post interrupt
float smoothFactor3 = 0.9; // top: pre interrupt
// on received signal interrupt
ISR(PCINT2_vect) {
    TCCR1B = 0; // disable counting
    uint8_t isFallingEdge = (PINK & (1 << 5)) == 0; // falling edge of PCINT21/pin5


    captureCounter = TCNT1L;
    captureCounter |= (TCNT1H << 8);

    // 1st signal of a package
    if ((isFallingEdge) // start occurs only on falling edge
        && ((packageIdleClock > maxPackageIdleClocks) // and package was interrupted
            || (packageClock > maxPackageClocks) // or package exceeds max length
        )) {

        // TCCR1B = 0; // disable counting
        TCNT1H = 0;
        TCNT1L = 0;

        packageClock = 0;
        packageIdleClock = 0;
        // TCCR1B |= timerClk; // enable counting

        digitalWrite(syncOutPin, HIGH);
        //        digitalWrite(redLed, HIGH);
        digitalWrite(compTopOutPin, !digitalRead(compTopOutPin));
    }

        // receiving signal within a package
    else if ((centerWindowHandledAtCounter != packageClock) && // de-bouncing
             (packageIdleClock <= maxPackageIdleClocks) && (packageClock <= maxPackageClocks)) {
        //        captureCounter = TCNT1L;
        //        captureCounter |= (TCNT1H << 8);

        packageIdleClock = 0;
        centerDelta = (compareA - compareB) / 2;
        topDelta = (centerDelta == 0) ? 0 : centerDelta - 1;

        // if signal occurs approx. at 1/2 of a package clock: the flank direction defines a data bit
        if (//(centerWindowHandledAtCounter != packageClock) &&
                ((compareB - centerDelta) <= captureCounter) && (captureCounter <= (compareB + centerDelta))) {

            int16_t offset = 2 * (captureCounter - compareB);
            packageIdleClock = 0;

            compareA += (offset * smoothFactor1);
            compareA = (compareA < minCompareA) ? pkgApproxClkCounter : compareA;
            compareA = (compareA > maxCompareA) ? pkgApproxClkCounter : compareA;

            // TCCR1B = 0;
            OCR1AH = (0xFF00 & compareA) >> 8;
            OCR1AL = 0x00FF & compareA;
            // TCCR1B |= timerClk;

            // remember for which package clock this window was handled - de-bouncing signal
            centerWindowHandledAtCounter = packageClock;
            topWindowHandledAtCounter = 0xFF;
        }

            // if signal occurs approx. at the end of the signal clock
        else if ((topWindowHandledAtCounter != packageClock) && // de-bouncing
                 (((compareA - topDelta) <= captureCounter) && (captureCounter <= (compareA + topDelta)) ||
                  (captureCounter <= topDelta))) {

            // TCCR1B = 0; // disable counting

            // remember for which package clock this window was handled - de-bouncing signal
            centerWindowHandledAtCounter = 0xFF;
            topWindowHandledAtCounter = packageClock;

            packageIdleClock = 0;

            // compareA interrupt has occurred previously
            if (captureCounter <= topDelta) {
                compareA = compareA + (captureCounter * smoothFactor2);
            } else // approaching compareA interrupt is skipped by now
            {
                compareA = captureCounter + ((captureCounter - compareA) * smoothFactor3);
                packageClock = (packageClock < 0xFFFF) ? packageClock + 1 : 0xFFFF;

                // reset de-bouncing marks
                centerWindowHandledAtCounter = 0xFF;
                topWindowHandledAtCounter = 0xFF;

                digitalWrite(yellowLed, !digitalRead(yellowLed));
                digitalWrite(compTopOutPin, !digitalRead(compTopOutPin));
            }

            compareA = (compareA < minCompareA) ? pkgApproxClkCounter : compareA;
            compareA = (compareA > maxCompareA) ? pkgApproxClkCounter : compareA;
            compareB = compareA / 2;

            TCNT1H = 0;
            TCNT1L = 0;

            OCR1AH = (0xFF00 & compareA) >> 8;
            OCR1AL = 0x00FF & compareA;

            OCR1BH = (0xFF00 & compareB) >> 8;
            OCR1BL = 0x00FF & compareB;
            // TCCR1B |= timerClk; // enable counting

        }
        // un-classify-able signal
        //        else {
        //            Serial.print(" M @ ");
        //            Serial.print(captureCounter);
        //
        //            if (topWindowHandledAtCounter == packageClock) {
        //                Serial.print(" bouncT");
        //            }
        //
        //            if (centerWindowHandledAtCounter == packageClock) {
        //                Serial.print(" bouncC ");
        //            }
        //
        //            Serial.print(" B ");
        //            Serial.print(compareB);
        //            Serial.print(" A ");
        //            Serial.print(compareA);
        //            Serial.print(" p ");
        //            Serial.print(packageClock);
        //        }
    }
    TIFR1 = 0;  // clear pending ICF1 OCF1C OCF1B OCF1A TOV1 interrupts
    TCCR1B |= timerClk; // enable counting
}


// the setup routine runs once when you press reset:
void setup() {

    //    Serial.begin(115200);
    Serial.begin(19200);
    Serial.println("\n\nrst");
    Serial.print("approx. top ");
    Serial.println(pkgApproxClkCounter);
    Serial.print("approx. center ");
    Serial.println(compareB);
    Serial.print("max pkg drift ");
    Serial.println(maxPkgClkDrift);
    Serial.print("min top ");
    Serial.println(minCompareA);
    Serial.print("max top  ");
    Serial.println(maxCompareA);

    cli();
    pinMode(redLed, OUTPUT);
    digitalWrite(redLed, LOW);
    pinMode(yellowLed, OUTPUT);
    digitalWrite(yellowLed, LOW);

    pinMode(leftMotor, OUTPUT);
    digitalWrite(leftMotor, LOW);
    pinMode(rightMotor, OUTPUT);
    digitalWrite(rightMotor, LOW);

    //    pinMode(compCenterOutPin, OUTPUT);
    //    digitalWrite(compCenterOutPin, LOW);
    pinMode(compTopOutPin, OUTPUT);
    digitalWrite(compTopOutPin, LOW);
    pinMode(syncOutPin, OUTPUT);
    digitalWrite(syncOutPin, LOW);

    pinMode(topInaccurateOutPin, OUTPUT);
    digitalWrite(topInaccurateOutPin, LOW);


    TCNT1H = 0;
    TCNT1L = 0;

    OCR1AH = (0xFF00 & compareA) >> 8;
    OCR1AL = 0x00FF & compareA;
    OCR1BH = (0xFF00 & compareB) >> 8;
    OCR1BL = 0x00FF & compareB;

    TCCR1A = 0;
    // disconnect output compare port operation
    TCCR1A |= (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << COM1C1) | (0 << COM1C0);
    // wave generator mode: normal
    TCCR1A |= (0 << WGM11) | (0 << WGM10);

    TCCR1B = 0;
    // input compare noise canelling: off
    TCCR1B |= (0 << ICNC1) | (0 << ICES1);
    // wave generator mode: normal
    TCCR1B |= (0 << WGM13) | (0 << WGM12);
    // counter prescaler: disconnected
    TCCR1B |= (0 << CS12) | (0 << CS11) | (0 << CS10);

    TIMSK1 = 0;
    // output compare interrupt enable
    TIMSK1 |= (0 << ICIE1) | (0 << OCIE1C) | (0 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1);

    pinMode(inputSignalPin, INPUT);
    digitalWrite(inputSignalPin, HIGH);

    // enable PCINT[23:16] interrupt on port K
    PCICR |= (1 << PCIE2);
    PCMSK2 = 0;
    // enable interrupt PCINT21 to contribute
    PCMSK2 |= (1 << PCINT21);

    sei();
    TCCR1B |= timerClk; // enable counting
}


void loop() {

    /*
    int8_t printCounter = 0;
    while (1) {
         int x = analogRead(joystickXPin);
         if (x >= 700) {
         counterTopOffset = 1000;
         }
         else if (x <= 300) {
         counterTopOffset = -1000;
         }
         else {
         counterTopOffset = 0;
         }

         int y = analogRead(joystickYPin);
         if (y >= 700) {
         counterCenterOffset = 1000;
         }
         else if (y <= 300) {
         counterCenterOffset = -1000;
         }
         else {
         counterCenterOffset = 0;
         }

         delay(250);
         counterCenterOffset = 0;
         counterTopOffset = 0;

         delay(1000);
    */
    /*
        if ((printCounter++ % 4) == 0) {
            Serial.print("top ");
            Serial.print(compareA);
            Serial.print(" center ");
            Serial.println(compareB);
        }
        delay(250);
    }
    */
}

