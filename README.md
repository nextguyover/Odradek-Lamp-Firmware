# Odradek Lamp Firmware

This repository contains firmware for the Arduino Nano to control an LED driver. This program was written for a desk lamp based on the Odradek terrain scanner from the game Death Stranding. For more information, see [this blog post](https://insertnewline.com/blog/led-driver-death-stranding-lamp).

# Usage

To flash this firmware simply open the `odradek_firmware.ino` file in the Arduino IDE, install the [required libraries](#required-libraries), then compile and flash. 

## IR Codes

The firmware supports controlling the lamp brightness using an IR remote. You will need to set the IR remote codes for your own specific remote. To do this, first flash an example sketch of the [IRremote](https://www.arduino.cc/reference/en/libraries/irremote/) library that prints any received IR codes to the Serial console. Using this, you should be able to get the IR codes for the buttons on your remote that you want to map lamp functionality to.

Once you have your codes, replace the existing codes in `odradek_firmware.ino` with your own codes.

```cpp
// odradek_firmware.ino

const char ir_code_power = 0x59;

const char ir_code_led_1_up = 0x06;
const char ir_code_led_1_down = 0x16;
const char ir_code_led_2_up = 0x1B;
const char ir_code_led_2_down = 0x5A;
const char ir_code_brightness_up = 0x0E;
const char ir_code_brightness_down = 0x12;

const char ir_code_preset_assign = 0x17;
const char ir_code_preset_slots[] = {0x52, 0x50, 0x10, 0x56, 0x54, 0x14, 0x4E, 0x4C, 0x0C, 0x0F};
```

The `ir_code_preset_slots` should contain a list of codes that corresponds to buttons on your remote that you wish to use as preset slots, e.g. each number on the remote number pad.

## Required Libraries

- [IRremote](https://www.arduino.cc/reference/en/libraries/irremote/)
- [TimerFreeTone](https://bitbucket.org/teckel12/arduino-timer-free-tone/wiki/Home)