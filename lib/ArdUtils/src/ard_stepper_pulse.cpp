#include <Arduino.h>

#include "ard_stepper.h"

#ifdef ARD_STEPPER_PULSE_ENABLED

#ifdef __AVR__

#include <TimerOne.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#endif

// pulse train

volatile ArdStepperState s_pulse_state;
volatile ArdStepperPins s_pins;

static void IRAM_ATTR ard_stepper_isr_pulse_train();

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us);
static void ard_stepper_pulse_timer_update(const uint32_t step_size_us);

static void ard_stepper_pulse_timer_disable();
static void ard_stepper_pulse_timer_enable();

static void ard_stepper_guard_on();
static void ard_stepper_guard_off();

#ifdef __AVR__

// avr implementation
// ------------------

static void ard_stepper_isr_pulse_train()
{
    // apply pulse train
    if (s_pulse_state.steps_remaining > 0)
    {
        // send pulse
        digitalWrite(s_pins.pulse, HIGH);
        // reset pulse
        delayMicroseconds(ARD_STEPPER_PULSE_DURATION_MICROSECONDS);
        digitalWrite(s_pins.pulse, LOW);
        // decrement remaining steps and advance timing
        s_pulse_state.steps_remaining -= 1;
    }
}

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us)
{
    // use TimeOne library for AVR

    Timer1.initialize(step_size_us);
    Timer1.attachInterrupt(ard_stepper_isr_pulse_train);
    Timer1.stop();
}

static void ard_stepper_pulse_timer_update(const uint32_t step_size_us)
{
    Timer1.setPeriod(step_size_us);
}

static void ard_stepper_pulse_timer_disable()
{
    s_pulse_state.enabled = false;
    digitalWrite(s_pins.enable, HIGH);
    Timer1.stop();
}

static void ard_stepper_pulse_timer_enable()
{
    Timer1.start();
    digitalWrite(s_pins.enable, LOW);
    s_pulse_state.enabled = true;
}

static void ard_stepper_guard_on() { noInterrupts(); }

static void ard_stepper_guard_off() { interrupts(); }

#else

// ESP32 implementation
// --------------------

hw_timer_t *s_timer = NULL;
portMUX_TYPE s_timer_mutex = portMUX_INITIALIZER_UNLOCKED;

static void ard_stepper_guard_on() { portENTER_CRITICAL(&s_timer_mutex); }

static void ard_stepper_guard_off() { portEXIT_CRITICAL(&s_timer_mutex); }

static void IRAM_ATTR ard_stepper_isr_pulse_train()
{
    portENTER_CRITICAL_ISR(&s_timer_mutex);
    // apply pulse train
    if (s_pulse_state.steps_remaining > 0)
    {
        // send pulse
        digitalWrite(s_pins.pulse, HIGH);
        // reset pulse
        delayMicroseconds(ARD_STEPPER_PULSE_DURATION_MICROSECONDS);
        digitalWrite(s_pins.pulse, LOW);
        // decrement remaining steps and advance timing
        s_pulse_state.steps_remaining -= 1;
    }
    portEXIT_CRITICAL_ISR(&s_timer_mutex);
}

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us)
{
    // use timerBegin with timeout for ESP32
    s_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(s_timer, &ard_stepper_isr_pulse_train, true);
    timerAlarmWrite(s_timer, step_size_us, true);
    timerAlarmEnable(s_timer);
}

static void ard_stepper_pulse_timer_update(const uint32_t step_size_us)
{
    // use timerBegin with timeout for ESP32
    timerAlarmWrite(s_timer, step_size_us, true);
}

static void ard_stepper_pulse_timer_enable()
{
    // use timerBegin with timeout for ESP32
    timerAlarmEnable(s_timer);
    digitalWrite(s_pins.enable, LOW);
    s_pulse_state.enabled = true;
}

static void ard_stepper_pulse_timer_disable()
{
    // use timerBegin with timeout for ESP32
    s_pulse_state.enabled = false;
    digitalWrite(s_pins.enable, HIGH);
    timerAlarmDisable(s_timer);
}

#endif

void ard_stepper_pulse_train_reset(const bool enable)
{
    ard_stepper_guard_on();

    // pulse state
    s_pulse_state.pulse_step_size = 0;
    s_pulse_state.steps_remaining = 0;
    s_pulse_state.direction = 0;

    digitalWrite(s_pins.pulse, LOW);
    if (enable)
    {
        // turn on: ready for pulse train
        ard_stepper_pulse_timer_enable();
    }
    else
    {
        // turn off pulse train
        ard_stepper_pulse_timer_disable();
    }
    
    ard_stepper_guard_off();
}

void ard_stepper_pulse_train_setup(const ArdStepperPins *pins,
                                   const uint32_t pulse_timer_step_size_us)
{
    ard_stepper_guard_on();

    // pulse state
    s_pulse_state.pulse_step_size = 0;
    s_pulse_state.steps_remaining = 0;
    s_pulse_state.direction = 0;

    // pins
    s_pins.enable = pins->enable;
    s_pins.direction = pins->direction;
    s_pins.direction_polarity = pins->direction_polarity;
    s_pins.pulse = pins->pulse;

    pinMode(s_pins.pulse, OUTPUT);
    digitalWrite(s_pins.pulse, LOW);

    pinMode(s_pins.enable, OUTPUT);
    digitalWrite(s_pins.enable, HIGH);

    pinMode(s_pins.pulse, OUTPUT);
    digitalWrite(s_pins.pulse, LOW);

    pinMode(s_pins.direction, OUTPUT);
    digitalWrite(s_pins.direction, (s_pins.direction_polarity ? HIGH : LOW));

    // pulse timer
    ard_stepper_pulse_timer_setup(pulse_timer_step_size_us);

    ard_stepper_guard_off();
}

void IRAM_ATTR ard_stepper_consume_run(ArdStepper *stepper, const uint32_t now)
{
    if (stepper->enabled)
    {
        // get next command number of pulses

        // space available in buffer
        size_t queued_positions =
            (stepper->buffer_tail > stepper->buffer_index
                 ? (stepper->buffer_size - stepper->buffer_tail) + stepper->buffer_index
                 : stepper->buffer_index - stepper->buffer_tail);
        // consume next available command
        if (queued_positions > 0)
        {
            // consume at tail of queue
            const int32_t delta = stepper->buffer[stepper->buffer_tail];
            // advance
            stepper->buffer_tail = (stepper->buffer_tail + 1) % stepper->buffer_size;

            // convert command to pulse train
            if (delta != 0)
            {
                // check steps remaining
                ard_stepper_guard_on();
                uint32_t steps_remaining = s_pulse_state.steps_remaining;
                const uint32_t pulse_step_size = s_pulse_state.pulse_step_size;
                ard_stepper_guard_off();

                // wait for steps remaining
                // this should not happen often, and wait should be short if timing is respected.
                while (steps_remaining > 0)
                {
                    // wait to finish
                    delayMicroseconds(pulse_step_size * steps_remaining);
                    // check steps remaining
                    ard_stepper_guard_on();
                    steps_remaining = s_pulse_state.steps_remaining;
                    ard_stepper_guard_off();
                }

                // modify pulse state
                ard_stepper_guard_on();

                // direction
                if (delta < 0)
                {
                    s_pulse_state.steps_remaining = -delta;
                    s_pulse_state.direction = s_pins.direction_polarity ? LOW : HIGH;
                }
                else
                {
                    s_pulse_state.steps_remaining = delta;
                    s_pulse_state.direction = s_pins.direction_polarity ? HIGH : LOW;
                }
                // set pulse steps
                s_pulse_state.pulse_step_size =
                    stepper->command_step_size / s_pulse_state.steps_remaining;

                // set direction pin
                digitalWrite(s_pins.direction, s_pulse_state.direction);

                // ensure drive is enabled
                if (!s_pulse_state.enabled)
                {
                    ard_stepper_pulse_timer_enable();
                }

                // update pulse timer step size
                ard_stepper_pulse_timer_update(s_pulse_state.pulse_step_size);

                // resume pulse
                ard_stepper_guard_off();
            }
        }
        else
        {
            ard_stepper_guard_on();
            if (s_pulse_state.steps_remaining == 0)
            {
                // no more commands waiting, no steps: turn off pulse train
                ard_stepper_pulse_timer_disable();
            }
            ard_stepper_guard_off();
        }
    }
}

#endif

// // Arduino timer CTC interrupt example
// //
// // avr-libc library includes
// #include <avr/io.h>
// #include <avr/interrupt.h>
// #define LEDPIN 13
// #define pulse 7

// int grad = 4;
// int b = OCR1A;
// long previousMillis = 0;
// long interval = 1000;
// unsigned long currentMillis = millis();
// int ledState = LOW;

// void setup()
// {
//  Serial.begin(9600);
//  pinMode(pulse, OUTPUT);
//  digitalWrite(pulse, LOW);
// pinMode(LEDPIN, OUTPUT);
// digitalWrite(LEDPIN, LOW);
// attachInterrupt(0, start_timer, FALLING);

// }

// void loop()
// {
// unsigned long currentMillis = millis();
//  if(currentMillis - previousMillis > interval) {
//    previousMillis = currentMillis; // save the last time you blinked the LED
//    if (ledState == LOW)// if the LED is off turn it on and vice-versa:
//      ledState = HIGH;
//    else
//      ledState = LOW;
//    digitalWrite(pulse, ledState); // set the LED with the ledState of the variable:
//  }
// }

// void start_timer()
// {
// digitalWrite(LEDPIN, HIGH);
// cli();          // disable global interrupts
// TCNT1 = 0;
// TCCR1A = 0;     // set entire TCCR1A register to 0
// TCCR1B = 0;     // same for TCCR1B
// OCR1A = 62; //3.7 mS
// //OCR1A = 11; //432 mS
// TCCR1B |= (1 << WGM12); // turn on CTC mode:
// TCCR1B |= (1 << CS10); // Set CS10 and CS12 bits for 1024 prescaler:
// TCCR1B |= (1 << CS12);
// TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt:
// sei(); // enable global interrupts:
// }

// ISR(TIMER1_COMPA_vect)
// {
// digitalWrite(LEDPIN, LOW);
// }
