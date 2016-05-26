# Manchester Coding - Proof of Concept
A manchester coding/decoding example using two Arduino Mega 2560

* transmitter: data_clock_provider
* receiver: timesync_concept

## Signal example
![signal-example](https://raw.githubusercontent.com/rubienr/manchester-coding-PoC/master/docs/example.jpg)
* CH1: receiver: received data
* CH2: receiver: PCINT2 rx handler duration
* CH3: receiver: TIMER1_COMPB_vect handler duration
* CH4: receiver: TIMER1_COMPA_vect 
* CH5: sender: tx clock
* CH6: sender: clock XOR data
* CH7: receiver: reception clock
* CH8: receiver: isReceiving == true
