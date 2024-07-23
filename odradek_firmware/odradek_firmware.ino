// https://github.com/nextguyover/Odradek-Lamp-Firmware

#include <Arduino.h>
#include <EEPROM.h>
#include <TimerFreeTone.h>

#include <IRremote.hpp>

#define pot_pin_1 A0  // potentiometer pins
#define pot_pin_2 A1

#define led_pin_warm 9  // LED output pins (PWM)
#define led_pin_cold 10

#define ir_recv_pin 4  // IR receiver pin

#define buzzer_pin 12  // buzzer pin

#define invert_pwm true  // set to true if LED is on when PWM duty cycle is 0

#define analog_read_hysteresis \
    5  // only update brightness if potentiometer value changes by this amount (prevents brightness oscillations due to
       // ADC noise)
#define analog_read_recouple_hysteresis \
    20  // potentiometer value change threshold to recouple potentiometers after decoupling

// brightness changes are animated between current value and target value, configure animation speed here
#define brightness_change_factor 10   // higher values make brightness animate change faster
#define brightness_min_change_rate 1  // sets lower limit on brightness change rate
#define brightness_update_delay 10    // time in ms between brightness updates, lower values for faster animations

#define ir_recv_deadtime \
    150  // set this to just over the time your remote takes to send repeat codes while holding down a button

#define ir_brightness_change_increment 1       // brightness increment on single button press
#define ir_brightness_change_increment_fast 2  // brightness increment multiplier when holding down a button

#define preset_awaiting_slot_timeout 10000  // time in ms to wait for slot selection after pressing preset assign button

//----------- Set your IR remote codes here ----------//

const char ir_code_power = 0x59;

const char ir_code_led_1_up = 0x06;
const char ir_code_led_1_down = 0x16;
const char ir_code_led_2_up = 0x1B;
const char ir_code_led_2_down = 0x5A;
const char ir_code_brightness_up = 0x0E;
const char ir_code_brightness_down = 0x12;

const char ir_code_preset_assign = 0x17;
const char ir_code_preset_slots[] = {0x52, 0x50, 0x10, 0x56, 0x54, 0x14, 0x4E, 0x4C, 0x0C, 0x0F};

//----------------------------------------------------//

unsigned long irPrevTime = 0;
unsigned long brightnessUpdatePrevTime = 0;

char irRepeatCode = 0x00;
unsigned long irRepeatCodeStartTime = 0;
int irRepeatCodeCount = 0;

int potReadPrev1, potReadPrev2 = 0;

int brightnessTarget1 = invert_pwm ? 255 : 0;
int brightnessTarget2 = invert_pwm ? 255 : 0;

float brightnessCurrent1 = invert_pwm ? 255 : 0;
float brightnessCurrent2 = invert_pwm ? 255 : 0;

bool brightnessPotCoupled = true;  // start off with brightness set by potentiometers

bool powerState = true;

int potDecoupleVal1 = -1;
int potDecoupleVal2 = -1;

int irBrightnessSaved1 = 0;
int irBrightnessSaved2 = 0;

int potRead1;
int potRead2;

enum PRESET_MODE { STANDBY, AWAITING_SLOT, SET_SUCCESS };
PRESET_MODE presetSetter = STANDBY;
unsigned long presetAwaitingStartTime = 0;

unsigned long buzzerTickTockTime = 0;
bool buzzerTick = true;

void setup() {
    Serial.begin(115200);

    pinMode(led_pin_warm, OUTPUT);
    pinMode(led_pin_cold, OUTPUT);

    pinMode(buzzer_pin, OUTPUT);

    IrReceiver.begin(ir_recv_pin);

    // configure PWM frequency of 62.5kHz
    TCCR1A = (1 << COM1A1) | (1 << WGM11);               // Configure OC1A for Fast PWM
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // Set Fast PWM mode and no prescaling
    ICR1 = 255;                                          // Set TOP value for 8-bit resolution (255 for max frequency)
}

void loop() {
    // read potentiometers
    potRead1 = analogRead(pot_pin_1);
    potRead2 = analogRead(pot_pin_2);

    // calculate brightness target from potentiometer values
    if (brightnessPotCoupled &&
        (abs(potRead1 - potReadPrev1) > analog_read_hysteresis || potRead1 == 0 || potRead1 == 1023)) {
        potReadPrev1 = potRead1;
        brightnessTarget1 = map(potRead1, 0, 1023, 0, 255);
    }
    if (brightnessPotCoupled &&
        (abs(potRead2 - potReadPrev2) > analog_read_hysteresis || potRead2 == 0 || potRead2 == 1023)) {
        potReadPrev2 = potRead2;
        brightnessTarget2 = map(potRead2, 0, 1023, 0, 255);
    }

    // if currently decoupled, check whether pots have moved away from decoupled values enough to recouple
    if (!brightnessPotCoupled && (abs(potRead1 - potDecoupleVal1) > analog_read_recouple_hysteresis ||
                                  abs(potRead2 - potDecoupleVal2) > analog_read_recouple_hysteresis)) {
        powerState = true;
        brightnessPotCoupled = true;
        potReadPrev1 = -100;  // negative values to force refresh
        potReadPrev2 = -100;
    }

    // process IR codes
    if (IrReceiver.decode()) {
        Serial.print("Received IR code: ");
        Serial.println(IrReceiver.decodedIRData.command, HEX);

        unsigned long currentMillis = millis();
        if ((unsigned long)(currentMillis - irPrevTime) >= ir_recv_deadtime)  // this section for ir codes that
                                                                              // shouldn't repeat
        {
            switch (IrReceiver.decodedIRData.command) {
                case ir_code_power:
                    decouplePotentiometers();

                    powerState = !powerState;

                    if (!powerState && brightnessTarget1 == 0 &&
                        brightnessTarget2 == 0) {  // if power turn off received, and brightness
                                                   // is already 0, instead do power turn on
                        powerState = true;
                        irBrightnessSaved1 = 30;
                        irBrightnessSaved2 = 30;
                    }

                    if (powerState) {                            // if switching off
                        brightnessTarget1 = irBrightnessSaved1;  // save current brightness
                        brightnessTarget2 = irBrightnessSaved2;
                    } else {                                     // if switching on
                        irBrightnessSaved1 = brightnessTarget1;  // restore from saved brightness
                        irBrightnessSaved2 = brightnessTarget2;

                        brightnessTarget1 = 0;
                        brightnessTarget2 = 0;
                    }
                    break;
                case ir_code_preset_assign:
                    if (presetSetter != AWAITING_SLOT) {
                        presetSetter = AWAITING_SLOT;
                        presetAwaitingStartTime = millis();
                    } else {
                        presetSetter = STANDBY;
                        buzzerTickTockTime = 0;
                    }
                    break;
                default:
                    for (int i = 0; i < sizeof(ir_code_preset_slots); i++) {  // check for preset slot codes
                        if (IrReceiver.decodedIRData.command == ir_code_preset_slots[i]) {
                            if (presetSetter == AWAITING_SLOT) {          // if currently awaiting a slot
                                EEPROM.update(2 * i, brightnessTarget1);  // save the current brightness to the slot
                                EEPROM.update(2 * i + 1, brightnessTarget2);
                                presetSetter = SET_SUCCESS;
                            } else {  // if not awaiting a slot, read from the slot and set brightness to preset value
                                decouplePotentiometers();
                                powerState = true;
                                brightnessTarget1 = EEPROM.read(2 * i);
                                brightnessTarget2 = EEPROM.read(2 * i + 1);
                            }
                        }
                    }
                    break;
            }
        }

        switch (IrReceiver.decodedIRData.command) {
            case ir_code_led_1_up:
            case ir_code_led_1_down:
            case ir_code_led_2_up:
            case ir_code_led_2_down:
            case ir_code_brightness_up:
            case ir_code_brightness_down:
                handleIrBrightnessChange();
        }

        brightnessTarget1 = constrain(brightnessTarget1, 0, 255);
        brightnessTarget2 = constrain(brightnessTarget2, 0, 255);

        irPrevTime = currentMillis;
        IrReceiver.resume();
    }

    // reset IR code repeat timer
    if ((unsigned long)(millis() - irRepeatCodeStartTime) >= ir_recv_deadtime) {
        irRepeatCode = 0x00;
        irRepeatCodeCount = 0;
    }

    handlePresets();

    updateBrightness();
}

/**
 * Handles preset setting timing and tone generation using buzzer.
 */
void handlePresets() {
    if (presetSetter == AWAITING_SLOT) {
        unsigned long currentMillis = millis();

        if ((unsigned long)(currentMillis - presetAwaitingStartTime) >= preset_awaiting_slot_timeout) {
            presetSetter = STANDBY;
            buzzerTickTockTime = 0;
        } else {
            if ((unsigned long)(currentMillis - buzzerTickTockTime) >= 500) {
                buzzerTickTockTime = currentMillis;

                if (buzzerTick) {  // play preset waiting tick tock tone on buzzer
                    TimerFreeTone(buzzer_pin, 500, 5);
                } else {
                    TimerFreeTone(buzzer_pin, 150, 7);
                }
                buzzerTick = !buzzerTick;
            }
        }
    }

    if (presetSetter == SET_SUCCESS) {  // play preset set success tone on buzzer
        TimerFreeTone(buzzer_pin, 500, 20);
        delay(100);
        TimerFreeTone(buzzer_pin, 1000, 50);

        presetSetter = STANDBY;
        buzzerTickTockTime = 0;
    }
}

/**
 * Updates LED brightness based on target values and set PWM duty cycle.
 */
void updateBrightness() {
    unsigned long currentMillis = millis();
    if ((unsigned long)(currentMillis - brightnessUpdatePrevTime) >= brightness_update_delay) {
        brightnessUpdatePrevTime = currentMillis;

        float brightnessDelta1 = (float)(brightnessTarget1 - brightnessCurrent1) / (float)brightness_change_factor;
        float brightnessDelta2 = (float)(brightnessTarget2 - brightnessCurrent2) / (float)brightness_change_factor;

        if (brightnessDelta1 != 0)
            brightnessCurrent1 +=
                max(abs(brightnessDelta1), (float)brightness_min_change_rate) * (brightnessDelta1 > 0 ? 1 : -1);
        if (brightnessDelta2 != 0)
            brightnessCurrent2 +=
                max(abs(brightnessDelta2), (float)brightness_min_change_rate) * (brightnessDelta2 > 0 ? 1 : -1);

        brightnessCurrent1 = constrain(brightnessCurrent1, 0, 255);
        brightnessCurrent2 = constrain(brightnessCurrent2, 0, 255);

        analogWrite(led_pin_warm, !invert_pwm ? (int)(brightnessCurrent1 + 0.5)
                                              : 255 - (int)(brightnessCurrent1 + 0.5));  // rounding to int
        analogWrite(led_pin_cold,
                    !invert_pwm ? (int)(brightnessCurrent2 + 0.5) : 255 - (int)(brightnessCurrent2 + 0.5));
    }
}

/**
 * Decouples potentiometers to allow setting brightness with IR remote.
 */
void decouplePotentiometers() {
    if (brightnessPotCoupled) {
        potDecoupleVal1 = potRead1;
        potDecoupleVal2 = potRead2;
        brightnessPotCoupled = false;
    }
}

/**
 * Handles brightness change commands from IR remote.
 */
void handleIrBrightnessChange() {
    decouplePotentiometers();
    powerState = true;

    int increment = ir_brightness_change_increment;  // assign default increment value

    if (IrReceiver.decodedIRData.command == irRepeatCode) {  // if repeat code received
        irRepeatCodeCount++;                                 // increment repeat code count
        increment =
            ir_brightness_change_increment_fast *
            irRepeatCodeCount;  // set new increment to fast increment multiplier, multiplied by repeat code count (this
                                // increases rate of brightness change the longer you hold down the remote button)
    } else {
        irRepeatCode = IrReceiver.decodedIRData.command;
        irRepeatCodeCount = 0;
    }
    irRepeatCodeStartTime = millis();

    switch (IrReceiver.decodedIRData.command) {
        case ir_code_led_1_up:
            brightnessTarget1 += increment;
            break;
        case ir_code_led_1_down:
            brightnessTarget1 -= increment;
            break;
        case ir_code_led_2_up:
            brightnessTarget2 += increment;
            break;
        case ir_code_led_2_down:
            brightnessTarget2 -= increment;
            break;
        case ir_code_brightness_up:
            brightnessTarget1 += increment;
            brightnessTarget2 += increment;
            break;
        case ir_code_brightness_down:
            brightnessTarget1 -= increment;
            brightnessTarget2 -= increment;
            break;
    }
}