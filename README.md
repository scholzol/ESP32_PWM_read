# ESP32_PWM_read

## Introduction
This is a small helper repo for testing the reading of RC PWM pulses with the ESP32-WROOM Devkit. It is mainly based on the work from this source
(https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb) 

## Idea
I used the GPIO_IN_REG register of the ESP32, which is the representation of the INPUT status of the GPIO Pins 0-31. This register will be read in an ISR when the status of a pin is changed. In the ISR the duration between rising and falling flanks will be calculated for the dedicated Pins. In comparison to the arduinos (Uno, Nano, ..) the Pin- Ports will be handled differently in the ESP32. Therefore this modification was necessary.