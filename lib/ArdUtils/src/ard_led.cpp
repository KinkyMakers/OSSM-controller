
#include "ard_led.h"

#include <Arduino.h>
#include <FastLED.h>

#include "string.h"

// Palettes
//-----------

DEFINE_GRADIENT_PALETTE(pride){
    0,   0,   0,   255,  // blue
    51,  0,   255, 0,    // green
    102, 255, 220, 0,    // bright yellow
    153, 255, 92,  0,    // bright orange
    204, 255, 0,   0,    // full red
    255, 255, 0,   255   // purple
};
static CRGBPalette16 s_pride_palette = pride;

DEFINE_GRADIENT_PALETTE(reverse_pride){
    0,   255, 0,   255,  // purple
    51,  255, 0,   0,    // full red
    102, 255, 92,  0,    // bright orange
    153, 255, 220, 0,    // bright yellow
    204, 0,   255, 0,    // green
    255, 0,   0,   255   // blue
};
static CRGBPalette16 s_reverse_pride_palette = reverse_pride;

#define ARD_LED_NUM_PALLETTES 8
static const CRGBPalette16 s_scale_to_palette[ARD_LED_NUM_PALLETTES] = {
    PartyColors_p, RainbowColors_p, CloudColors_p, LavaColors_p,
    OceanColors_p, ForestColors_p,  HeatColors_p,  s_pride_palette};

// Animations
//-----------

void ard_animation_rainbow(ArdLedArray *array, uint32_t step_size, uint8_t init)
{
    // https://github.com/atuline/FastLED-Demos/blob/master/rainbow_beat/rainbow_beat.ino
    // speed to bpm
    uint16_t bpm = (array->speed > 200 ? 200 : array->speed + 13);
    // Starting hue
    uint8_t beatA = beatsin8(bpm + 4, 0, 255);
    uint8_t beatB = beatsin8(bpm, 0, 255);
    // Use FastLED's fill_rainbow routine.
    fill_rainbow(array->leds, array->num_leds, (beatA + beatB) / 2, 8);
}

void ard_animation_twinkle(ArdLedArray *array, uint32_t step_size, uint8_t init)
{
    // https://github.com/atuline/FastLED-Demos/blob/master/fadein/fadein.ino

    // Palette definitions
    static CRGBPalette16 currentPalette = PartyColors_p;
    static CRGBPalette16 targetPalette = PartyColors_p;

    if (init)
    {
        // reset index
        array->index = 0;
    }

    // Every 100ms
    array->index += 1;
    if (step_size * array->index > 100000)
    {
        // set palette
        targetPalette =
            s_scale_to_palette[array->scale >= 0 ? array->scale % ARD_LED_NUM_PALLETTES : 0];
        // transition changes
        uint8_t maxChanges = 24;
        nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
        array->index = 0;
    }

    // The randomizer needs to be re-set each time through the loop in order for the 'random'
    // numbers to be the same each time through.
    random16_set_seed(535);

    const unsigned long now = millis();
    for (uint16_t i = 0; i < array->num_leds; i++)
    {
        // The random number for each 'i' will be the same every time.
        uint8_t fader = sin8(now / random8(10, 20));
        // Now, let's run it through the palette lookup.
        array->leds[i] = ColorFromPalette(currentPalette, i * 20, fader, LINEARBLEND);
    }

    // Re-randomizing the random number seed for other routines.
    random16_set_seed(now);
}



void ard_animation_powerup(ArdLedArray *array, uint32_t step_size, uint8_t init)
{
    if (array->scale >= array->threshold)
    {
        // max value
        for (uint16_t k = 0; k < array->num_leds; ++k)
        {
            // color from palette
            const uint8_t pallette_index = static_cast<uint8_t>((k * 255U) / array->num_leds);
            array->leds[k] = ColorFromPalette(s_pride_palette, pallette_index);
        }
    }
    else
    {
        // color from palette
        const uint8_t pallette_index = map(array->scale, 0, array->threshold, 0, 255);
        const CRGB color = ColorFromPalette(s_pride_palette, pallette_index);

        // turn on
        const uint16_t num_leds_on = map(array->scale, 0, array->threshold, 1, array->num_leds);
        for (uint16_t k = 0; k < num_leds_on; ++k)
        {
            array->leds[k] = color;
        }
        for (uint16_t k = num_leds_on; k < array->num_leds; ++k)
        {
            array->leds[k] = CRGB::Black;
        }
    }
}

void ard_animation_powerdown(ArdLedArray *array, uint32_t step_size, uint8_t init)
{
    if (array->scale >= array->threshold)
    {
        // max value
        for (uint16_t k = 0; k < array->num_leds; ++k)
        {
            // color from palette
            const uint8_t pallette_index = static_cast<uint8_t>((k * 255U) / array->num_leds);
            array->leds[k] = ColorFromPalette(s_reverse_pride_palette, pallette_index);
        }
    }
    else
    {
        // color from palette
        const uint8_t pallette_index = map(array->scale, 0, array->threshold, 0, 255);
        const CRGB color = ColorFromPalette(s_pride_palette, pallette_index);

        // turn off
        const uint16_t num_leds_off =
            array->num_leds - map(array->scale, 0, array->threshold, 1, array->num_leds);
        for (uint16_t k = 0; k < num_leds_off; ++k)
        {
            array->leds[k] = CRGB::Black;
        }
        for (uint16_t k = num_leds_off; k < array->num_leds; ++k)
        {
            array->leds[k] = color;
        }
    }
}

// int ard_led_set_array_sizes( ArdLedParameters *parameters) {

//     num_arrays = 0;
//     if (num_arrays == 0) return -1;

//     parameters->num_leds_in_arrays = (uint16_t *) calloc(num_arrays, sizeof(uint16_t));
//     if (parameters->num_leds_in_arrays == nullptr) return -2;

//     // all good
//     for (size_t k = 0; k < num_arrays; ++k) {
//         parameters->num_leds_in_arrays[k] = num_leds_in_arrays[k];
//     }
//     num_arrays = num_arrays;

//     return 0;
// }

// Allocate

int ard_led_alloc(ArdLed *led, const ArdLedParameters *parameters,
                  const uint16_t *num_leds_in_arrays, uint16_t num_arrays)
{
    if (ARD_LED_NUM <= 0) return -1;

    led->num_leds = 0;
    led->num_arrays = 0;

    led->arrays = nullptr;

    led->leds = (CRGB *)calloc(ARD_LED_NUM, sizeof(CRGB));
    if (led->leds == nullptr)
    {
        ard_led_free(led);
        return -1;
    }

    led->arrays = (ArdLedArray *)calloc(num_arrays, sizeof(ArdLedArray));
    if (led->arrays == nullptr)
    {
        ard_led_free(led);
        return -2;
    }

    led->num_leds = ARD_LED_NUM;
    led->num_arrays = num_arrays;

    // initialize arrays
    size_t index = 0;
    for (uint16_t k = 0; k < num_arrays; ++k)
    {
        if (index > ARD_LED_NUM)
        {
            ard_led_free(led);
            return -3;
        }
        led->arrays[k].leds = &led->leds[index];
        led->arrays[k].num_leds = num_leds_in_arrays[k];
        led->arrays[k].animation = nullptr;
        led->arrays[k].index = 0;
        led->arrays[k].color = CRGB(0, 0, 0);
        led->arrays[k].scale = 0;
        led->arrays[k].speed = 0;
        led->arrays[k].threshold = 0;

        index += num_leds_in_arrays[k];
    }

#ifndef ARD_LED_PIN
    FastLED.addLeds<ARD_LED_CHIPSET>(led->leds, ARD_LED_NUM);
#else
#ifndef ARD_LED_CLOCK_PIN
    FastLED.addLeds<ARD_LED_CHIPSET, ARD_LED_PIN, ARD_LED_ORDER>(led->leds, ARD_LED_NUM);
#else
    FastLED.addLeds<ARD_LED_CHIPSET, ARD_LED_PIN, ARD_LED_CLOCK_PIN, ARD_LED_ORDER>(led->leds, ARD_LED_NUM);
#endif
#endif

    FastLED.setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(parameters->brightness);
    FastLED.setMaxPowerInVoltsAndMilliamps(parameters->volts, parameters->milliamps);

    return 0;
}

void ard_led_free(ArdLed *led)
{
    FastLED.clearData();

    free(led->arrays);
    led->arrays = nullptr;

    led->num_leds = 0;
    led->num_arrays = 0;
}

void ard_led_set_brightness(const uint8_t brightness) { FastLED.setBrightness(brightness); }

/**
 * @brief   Set animation
 *
 * @param     led           LED container
 * @param     array         Array index
 * @param     animation     DESCRIPTION
 * @param     color         DESCRIPTION
 * @param     scale         DESCRIPTION
 * @param     speed         DESCRIPTION
 * @return    int           DESCRIPTION
 */
void ard_led_array_set_animation(ArdLed *led, const uint16_t array, ArdLedArrayAnimation animation,
                                 const CRGB color, const int16_t scale, const int16_t speed, const int16_t threshold)
{
    if (array < led->num_arrays)
    {
        led->arrays[array].animation = animation;
        led->arrays[array].index = 0;
        led->arrays[array].color = color;
        led->arrays[array].scale = scale;
        led->arrays[array].speed = speed;
        led->arrays[array].threshold = threshold;
    }
}

void ard_led_array_set_animation_index(ArdLed *led, const uint16_t array, const uint16_t index)
{
    if (array < led->num_arrays)
    {
        if (index < led->arrays[array].num_leds)
        {
            led->arrays[array].index = index;
        }
    }
}

void ard_led_array_set_animation_scale(ArdLed *led, const uint16_t array, const int16_t scale)
{
    if (array < led->num_arrays)
    {
        led->arrays[array].scale = scale;
    }
}

void ard_led_array_set_animation_color(ArdLed *led, const uint16_t array, const CRGB color)
{
    if (array < led->num_arrays)
    {
        led->arrays[array].color = color;
    }
}

void ard_led_array_set_animation_speed(ArdLed *led, const uint16_t array, const int16_t speed)
{
    if (array < led->num_arrays)
    {
        led->arrays[array].speed = speed;
    }
}

void ard_led_array_set_animation_threshold(ArdLed *led, const uint16_t array,
                                           const int16_t threshold)
{
    if (array < led->num_arrays)
    {
        led->arrays[array].threshold = threshold;
    }
}

void ard_led_set_color(ArdLed *led, const CRGB color)
{
    for (uint16_t k = 0; k < led->num_leds; ++k)
    {
        led->leds[k] = color;
    }
}

void ard_led_initialize(ArdLed *led)
{
    // run animations
    for (uint16_t k = 0; k < led->num_arrays; ++k)
    {
        led->arrays[k].index = 0;
        if (led->arrays[k].animation != nullptr)
        {
            // animate
            led->arrays[k].animation(&led->arrays[k], led->step_size, true);
            led->arrays[k].index = 0;
        }
    }
}

void ard_led_reset(ArdLed *led, const CRGB color, const uint32_t now)
{
    // first set all to color
    ard_led_set_color(led, color);
    for (uint16_t k = 0; k < led->num_arrays; ++k)
    {
        // initialize animation for each array
        led->arrays[k].index = 0;
        if (led->arrays[k].animation != nullptr)
        {
            led->arrays[k].animation(&led->arrays[k], led->step_size, false);
        }
    }
    // initialize time
    led->prev_step = now;
    // show
    FastLED.show();
}

bool ard_led_run(ArdLed *led, const uint32_t now)
{
    bool ret = false;
    if ((now - led->prev_step) >= led->step_size)
    {
        // we are updating
        ret = true;
        // run animations
        for (uint16_t k = 0; k < led->num_arrays; ++k)
        {
            if (led->arrays[k].animation != nullptr)
            {
                // animate
                led->arrays[k].animation(&led->arrays[k], led->step_size, false);
            }
        }
        // advance
        led->prev_step += led->step_size;
        // show
        FastLED.show();
    }
    return ret;
}

void ard_led_white(ArdLed *led)
{
    ard_led_set_color(led, CRGB::White);
    ard_led_show();
}

void ard_led_black(ArdLed *led)
{
    ard_led_set_color(led, CRGB::Black);
    ard_led_show();
}

void ard_led_show() { FastLED.show(); }
