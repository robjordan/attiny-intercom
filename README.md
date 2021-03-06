# attiny-intercom
Wireless battery-powered intercom using ATtiny and NRF24L01+ radio.

The idea is to have a low-power, low-tech radio intercom for use between a pair of cyclists who sometimes get too far apart for face-to-face communications.

ATtiny 1 series microcontrollers have 10-bit ADC and 8-bit DAC which are fast enough for voice-grade 8kHz sampling => data rate 64 kb/s required.  Ideally we'd filter to 300Hz - 3.4KHz to avoid aliasing.

Cheap NRF24L01+ boards promise good range at 250kb/s (maybe 100 metres+?).

MAX9814 board with electret capsule gives a strong signal that's noisy but clear enough to be intelligible.

Here's a rough schematic of the solution:
![Rough schematic](./images/rough-schematic.jpg)

... and a photo of the prototype:
![Prototype](./images/prototype.jpg)




