# SuperBMS
 A versatile open source battery management system.

# Features
- Supports 4-20 Cells
- 24-bit ADCs
- Low quiescent current
- Customizable code (CircuitPython)
- Electrically isolated serial interface
- Flexible discharge resistor configuration
- Power control over charger and load
- 3-pin fan header
- Indicator buzzer and LEDs

## Measurement
The SuperBMS utilizes Texas Instruments' [ADS1248](https://www.ti.com/lit/ds/symlink/ads1248.pdf?ts=1596011900933&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1248) ADCs and [INA149](https://www.ti.com/lit/ds/symlink/ina149.pdf?ts=1596011892251&ref_url=https%253A%252F%252Fwww.google.com%252F) op-amps to provide between 20 and 22 bits (a few microvolts) of accuracy.
