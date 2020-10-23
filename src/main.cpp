#include <Arduino.h>
#include <Encoder.h>

#include "ard_timing.h"
#include "ard_led.h"
#include "ard_i2c.h"
#include "ard_mpu9250.h"
#include "ard_serial.h"
#include "ard_spline.h"
#include "ard_stepper.h"
#include "ard_mpu9250.h"

// Error
// -----

#define ERROR_ALLOCATE_MESSAGE "Failed to allocate memory"
#define ERROR_SPLINE_MESSAGE "Failed to generate spline"
#define ERROR_ENCODER_MESSAGE "Failed to configure encoder"
#define ERROR_MPU9250_MESSAGE "Failed to configure MPU9250"

static void fail_forever(const char *restrict msg)
{
    // prepare packet for write
    Serial.println(msg);
    while (true)
    {
        delay(3000);
        Serial.println(msg);
    }
}

static void fail_alloc() { fail_forever(ERROR_ALLOCATE_MESSAGE); }

// ArdTiming
// ----------

ArdTiming g_timing_10hz;
ArdTiming g_timing_100hz;
ArdTiming g_timing_200hz;

// Rotary Encoder
// --------------


#define ENCODER_BUTTON 25
#define ENCODER_PIN_CHA 27
#define ENCODER_PIN_CHB 26

#define ENCODER_RESOLUTION_BITS 32
#define ENCODER_COUNTS_PER_REV 72
#define ENCODER_VALUE_PER_REV 360.0

ArdEncoder g_encoder;
double g_encoder_angle;

Encoder g_encoder_quad(ENCODER_PIN_CHA, ENCODER_PIN_CHB);

int ard_encoder_set(ArdEncoder *enc, const uint8_t counter_resolution_bits,
                    const uint64_t counts_per_rev, const double value_per_rev);

static void encoder_configure() {
    
    const uint8_t counter_resolution_bits = ENCODER_RESOLUTION_BITS;
    const uint32_t counts_per_rev = ENCODER_COUNTS_PER_REV;
    
    int ret = ard_encoder_set(
        &g_encoder, 
        counter_resolution_bits, counts_per_rev, 
        ENCODER_VALUE_PER_REV);
    if (ret != 0) {
        fail_forever(ERROR_ENCODER_MESSAGE);
    }
}

// Serial send/receive
// -------------------

ArdSerialPacket g_packet_receive;
ArdSerialPacket g_packet_send;
ArdSerialPacket g_packet_error;

#define SERIAL_RECV_BUFFER_SIZE 32
#define SERIAL_RECV_DELIMITER '|'
#define SERIAL_RECV_USE_CRC true

#define SERIAL_SEND_BUFFER_SIZE 32
#define SERIAL_SEND_DELIMITER '$'
#define SERIAL_SEND_USE_CRC true

#define SERIAL_ERROR_BUFFER_SIZE 32
#define SERIAL_ERROR_DELIMITER '~'
#define SERIAL_ERROR_USE_CRC true

// MPU 9250
// --------

ArdMpu9250 g_imu;

// 200 Hz
#define IMU_RATE_DIVIDER 4

#define IMU_GYRO_BANDWIDTH EARD_MPU9250_GBAND_41HZ
#define IMU_GYRO_SCALE EARD_MPU9250_GFS_500DPS

#define IMU_ACC_BANDWIDTH EARD_MPU9250_ABAND_99HZ
#define IMU_ACC_SCALE EARD_MPU9250_AFS_8G

#define IMU_MAG_RATE EARD_MPU9250_MRATE_100HZ
#define IMU_MAG_PRECISION EARD_MPU9250_MPRECISION_16BITS

static void imu_configure(void) {
    
    ArdMpu9250Config config = ard_mpu9250_default_config();
    
    config.rate_divider = IMU_RATE_DIVIDER;
    config.accel_bandwidth = IMU_ACC_BANDWIDTH;
    config.gyro_bandwidth = IMU_GYRO_BANDWIDTH;
    config.accel_range = IMU_ACC_SCALE;
    config.gyro_range = IMU_GYRO_SCALE;
    config.mag_precision = IMU_MAG_PRECISION;
    config.mag_rate = IMU_MAG_RATE;
    
    eArdMpu9250ConfigResult result = ard_mpu9250_configure(&g_imu, &config);
    if (result != ARD_MPU9250_CONFIG_OK) {
        fail_forever(ERROR_MPU9250_MESSAGE);
    }
    delay(100);

}

// Limit Switches
// --------------

#define LIMIT_MIN_PIN 36
#define LIMIT_MAX_PIN 37
#define DEBOUNCE_READ_NUM 5
#define DEBOUNCE_READ_THRESHOLD 3

static void limits_setup() {
    pinMode(LIMIT_MIN_PIN, INPUT);
    pinMode(LIMIT_MAX_PIN, INPUT);
}

static bool read_limit_min() {
    return digitalRead(LIMIT_MIN_PIN);
}

static bool read_limit_max() {
    return digitalRead(LIMIT_MAX_PIN);
}

// LED
// ---

ArdLed g_led;
int g_led_animation_count = 0;
volatile uint32_t g_led_mode_last_interrupt = 0;
volatile uint8_t g_led_mode = 0;

#define LED_ANIMATION_STEPS 50
#define LED_ANIMATION_THRESHOLD 800
#define LED_ANIMATION_MAX_SCALE 1024

#define LED_NUM_ARRAYS 2
#define LED_LEDS_PER_ARRAY 20, 20
#define LED_BRIGHTNESS 150
#define LED_MILLIAMPS 500
#define LED_VOLTS 5

#ifndef ARD_LED_CHIPSET
#define ARD_LED_CHIPSET WS2812B
#ifndef ARD_LED_PIN
#define ARD_LED_PIN 11
#endif
#ifndef ARD_LED_ORDER
#define ARD_LED_ORDER GRB
#endif
#endif


#ifdef __AVR__
#define IRAM_ATTR
#endif

static void IRAM_ATTR led_isr_mode_change() { 
    const uint32_t now = millis();
    if ( (now - g_led_mode_last_interrupt) > 200) {
        g_led_mode = (g_led_mode + 1) % 3; 
    }
    g_led_mode_last_interrupt = now;
}

static uint8_t led_get_mode()
{
    noInterrupts();
    uint8_t mode = g_led_mode;
    interrupts();
    return mode;
}

static void led_mode_setup()
{
    noInterrupts();
    // on rising edge, set pin back to LOW after pulse width delay
    pinMode(ENCODER_BUTTON, INPUT_PULLUP);
#ifdef __AVR__
    attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON), led_isr_mode_change,
                    RISING);
#else
    attachInterrupt(ENCODER_BUTTON, led_isr_mode_change, FALLING);
#endif
    interrupts();
}

static ArdLedParameters led_setup(void)
{
    
    led_mode_setup();

    ArdLedParameters led_parameters;
    led_parameters.step_size = ARD_TIMING_50HZ_STEP_SIZE;
    led_parameters.brightness = LED_BRIGHTNESS;
    led_parameters.volts = LED_VOLTS;
    led_parameters.milliamps = LED_MILLIAMPS;

    return led_parameters;
}

// Stepper
// -------

#define STEPPER_MIN_PULSE_STEP_SIZE 100U
#define STEPPER_PULSES_PER_REV 6400
#define STEPPER_MAX_DEGREES_PER_SEC 180.0
#define STEPPER_COMMAND_DELAY_STEPS 2

#ifndef STEPPER_PIN_PULSE
#define STEPPER_PIN_PULSE 4
#endif

#ifndef STEPPER_PIN_DIRECTION
#define STEPPER_PIN_DIRECTION 5
#endif

#ifndef STEPPER_PIN_ENABLE
#define STEPPER_PIN_ENABLE 6
#endif

ArdStepper g_stepper;

static ArdStepperParameters get_stepper_parameters(void)
{
    ArdStepperParameters stepper_parameters;
    stepper_parameters.buffer_size = 16;
    stepper_parameters.pulses_per_rev = STEPPER_PULSES_PER_REV;
    stepper_parameters.units_per_rev = 360.0;
    stepper_parameters.max_unit_velocity = STEPPER_MAX_DEGREES_PER_SEC;
    stepper_parameters.min_pulse_step_size = STEPPER_MIN_PULSE_STEP_SIZE;
    stepper_parameters.command_step_size = ARD_TIMING_100HZ_STEP_SIZE;
    stepper_parameters.command_delay_steps = STEPPER_COMMAND_DELAY_STEPS;
    stepper_parameters.pins.pulse = STEPPER_PIN_PULSE;
    stepper_parameters.pins.direction = STEPPER_PIN_DIRECTION;
    stepper_parameters.pins.enable = STEPPER_PIN_ENABLE;
    stepper_parameters.pins.direction_polarity = 0;

    return stepper_parameters;
}

// Spline trajectory generator
// ---------------------------

#define TRAJ_SPLINE_START 0
#define TRAJ_SPLINE_END 180
#define TRAJ_SPLINE_DURATION_SECONDS 1.0

bool g_traj_forward;
ArdSplineTrajectory g_traj;
ArdSplinePva g_traj_start;
ArdSplinePva g_traj_end;

static void generate_test_traj(ArdSplineTrajectory *traj)
{
    g_traj_start.position = TRAJ_SPLINE_START;
    g_traj_start.velocity = 0;
    g_traj_start.acceleration = 0;

    g_traj_end.position = TRAJ_SPLINE_END;
    g_traj_end.velocity = 0;
    g_traj_end.acceleration = 0;

    int ret =
        ard_spline_traj_generate(&g_traj, &g_traj_start, &g_traj_end, TRAJ_SPLINE_DURATION_SECONDS,
                                 ARD_TIMING_100HZ_STEP_SIZE, EARD_SPLINE_QUINTIC);
    if (ret != 0) fail_forever(ERROR_SPLINE_MESSAGE);
    g_traj_forward = true;
}

// Test Pulse Counting
// -------------------

// test pin connected to stepper output
#ifndef STEPPER_TEST_PIN_PULSE
#define STEPPER_TEST_PIN_PULSE 2
#endif

// stepper test pulse counting

volatile uint32_t s_pulse_count = 0;

#ifdef __AVR__
#define IRAM_ATTR
#endif

static void IRAM_ATTR test_ard_isr_pulse_count() { s_pulse_count += 1; }

static uint32_t test_ard_get_pulses_counted()
{
    noInterrupts();
    uint32_t ret = s_pulse_count;
    interrupts();
    return ret;
}

static void test_ard_pulse_count_setup()
{
    noInterrupts();
    // on rising edge, set pin back to LOW after pulse width delay
    pinMode(STEPPER_TEST_PIN_PULSE, INPUT_PULLUP);
#ifdef __AVR__
    attachInterrupt(digitalPinToInterrupt(STEPPER_TEST_PIN_PULSE), test_ard_isr_pulse_count,
                    RISING);
#else
    attachInterrupt(STEPPER_TEST_PIN_PULSE, test_ard_isr_pulse_count, FALLING);
#endif
    interrupts();
}

// Setup
// -----

void setup()
{
    /*
        Start communications
        --------------------
    */

    // start serial
    ard_serial_begin(SERIAL_BAUDRATE);
    delay(500);

    // join i2c bus
    ard_i2c_master_begin();
    delay(100);
    
    // limits
    // -------
    
    limits_setup();

    // trajectory
    // ----------

    generate_test_traj(&g_traj);

    // ArdLed
    // ------

    ArdLedParameters led_parameters = led_setup();
    const uint16_t leds_in_arrays[LED_NUM_ARRAYS] = {LED_LEDS_PER_ARRAY};
    int ret = ard_led_alloc(&g_led, &led_parameters, leds_in_arrays, LED_NUM_ARRAYS);
    if (ret != 0) fail_alloc();
    // initialize animation
    ard_led_array_set_animation(&g_led, 0, ard_animation_powerup, CRGB::Black, 0, 0, LED_ANIMATION_THRESHOLD);
    ard_led_array_set_animation(&g_led, 1, ard_animation_powerdown, CRGB::Black, 0, 0, LED_ANIMATION_THRESHOLD);
    ard_led_initialize(&g_led);

    // ArdStepper
    // ----------

    // stepper test parameters
    ArdStepperParameters stepper_parameters = get_stepper_parameters();
    ret = ard_stepper_alloc(&g_stepper, &stepper_parameters);
    if (ret != 0) fail_alloc();

    // Test - count steps
    // ------------------

    encoder_configure();

    // Serial
    // ------

    ret = ard_packet_alloc(&g_packet_receive, SERIAL_RECV_BUFFER_SIZE, SERIAL_RECV_DELIMITER,
                           SERIAL_RECV_USE_CRC);
    if (ret != 0) fail_alloc();

    ret = ard_packet_alloc(&g_packet_send, SERIAL_SEND_BUFFER_SIZE, SERIAL_SEND_DELIMITER,
                           SERIAL_SEND_USE_CRC);
    if (ret != 0) fail_alloc();

    ard_packet_alloc(&g_packet_receive, SERIAL_ERROR_BUFFER_SIZE, SERIAL_ERROR_DELIMITER,
                     SERIAL_ERROR_USE_CRC);
    if (ret != 0) fail_alloc();

    // MPU 9250
    // --------
    
    // TODO(kescholm): Add MPU9250
    // imu_configure();

    // Timing
    // ------

    const uint32_t now = micros();
    ard_timing_reset(ARD_TIMING_10HZ_STEP_SIZE, now, &g_timing_10hz);
    ard_timing_reset(ARD_TIMING_100HZ_STEP_SIZE, now, &g_timing_100hz);
    ard_timing_reset(ARD_TIMING_200HZ_STEP_SIZE, now, &g_timing_200hz);

    // Finish Setup
    // ------------

    // Reset stepper
    ard_stepper_reset(&g_stepper, TRAJ_SPLINE_START, true);
}

void loop()
{
    const uint32_t now = micros();
    uint32_t elapsed_steps = 0;

    // 10 Hz
    // -----

    elapsed_steps = ard_timing_step(&g_timing_10hz, now);
    if (elapsed_steps)
    {
        // LED animation updates
        // ---------------------

        // set scale
        int scale = 0;
        const uint8_t mode = led_get_mode();
        if (mode == 0) {
            scale = map(g_led_animation_count, 0, LED_ANIMATION_STEPS, 0, LED_ANIMATION_MAX_SCALE);
        } else if (mode == 1) {
            //rainbow
            scale = map(constrain(g_encoder_angle, 0, ENCODER_VALUE_PER_REV), 0, ENCODER_VALUE_PER_REV, 0, LED_ANIMATION_MAX_SCALE);
        } else {
            //rainbow
            scale = LED_ANIMATION_THRESHOLD + 1;
        }

        // apply scale to aimation
        ard_led_array_set_animation_scale(&g_led, 0, scale);
        ard_led_array_set_animation_scale(&g_led, 1, scale);
        g_led_animation_count++;
        if (g_led_animation_count > LED_ANIMATION_STEPS)
        {
            g_led_animation_count -= LED_ANIMATION_STEPS;
        }
    }

    // run LEDs
    // --------

    ard_led_run(&g_led, now);

    // 100 Hz
    // ------

    elapsed_steps = ard_timing_step(&g_timing_100hz, now);
    if (elapsed_steps)
    {
        // spline trajectory
        // -----------------

        // make new reversed traj if end reached
        if (g_traj.index == g_traj.num_steps)
        {
            if (g_traj_forward)
            {
                ard_spline_traj_generate(&g_traj, &g_traj_end, &g_traj_start,
                                         TRAJ_SPLINE_DURATION_SECONDS, ARD_TIMING_100HZ_STEP_SIZE,
                                         EARD_SPLINE_QUINTIC);
                g_traj_forward = false;
            }
            else
            {
                ard_spline_traj_generate(&g_traj, &g_traj_start, &g_traj_end,
                                         TRAJ_SPLINE_DURATION_SECONDS, ARD_TIMING_100HZ_STEP_SIZE,
                                         EARD_SPLINE_QUINTIC);
                g_traj_forward = true;
            }
        }

        const double command = ard_spline_traj_evaluate_position(&g_traj);
        // produce test commands
        ard_stepper_produce_command(&g_stepper, &command, 1);

        // stepper
        // -------

        ard_stepper_consume_run(&g_stepper, now);
    }

    // 200 Hz
    // ------
    
    elapsed_steps = ard_timing_step(&g_timing_200hz, now);
    if (elapsed_steps)
    {

        // counting steps
        g_encoder_angle = ard_encoder_decode(&g_encoder, g_encoder_quad.read());
    }
}
