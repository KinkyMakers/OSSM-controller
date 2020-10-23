
/**
 * @file       ard_led.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Led controller
 *
 * @details
 *
 * See group @ref ArdLed
 *
 */

#ifndef ARD_LED_H
#define ARD_LED_H

#include <Arduino.h>

#include <SPI.h>

#include <FastLED.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus

#ifndef restrict
#define restrict __restrict
#endif

extern "C"
{
#endif

    /**
     * @defgroup   ArdLed LED Controller
     * @brief      Led controller
     *
     * @{
     *
     */

#ifndef ARD_LED_CHIPSET
#define ARD_LED_CHIPSET WS2803
#endif

#ifndef ARD_LED_NUM
#define ARD_LED_NUM 1
#endif
    
    typedef struct ArdLedParameters
    {
        /**
         * @brief Step size of LED animation update
         * 
         */
        uint32_t step_size;
        
        /**
         * @brief Brightness of entire strip
         * 
         */
        uint8_t brightness;

        /**
         * @brief LED power management volts
         * 
         */
        uint8_t volts;

        /**
         * @brief LED power management miliamps
         * 
         */
        uint32_t milliamps;

    } ArdLedParameters;

    typedef struct ArdLedArray
    {
        /**
         * @brief   LED array
         *
         */
        CRGB *leds;
        /**
         * @brief   number of LEDs
         *
         */
        uint16_t num_leds;

        /**
         * @brief   Animations callback
         *
         */
        void (*animation)(struct ArdLedArray *, uint32_t, uint8_t);

        /**
         * @brief   Current LED index in animation
         *
         */
        uint16_t index;
        /**
         * @brief   Color for animation (optional)
         *
         */
        CRGB color;
        /**
         * @brief   General purpose scale for animation (optional)
         *
         */
        int16_t scale;
        /**
         * @brief   Animation speed (optional)
         *
         */
        int16_t speed;
        /**
         * @brief   Animation threshold (optional)
         *
         */
        int16_t threshold;

    } ArdLedArray;

    /**
     * @brief   Array animation update callback
     *
     */
    typedef void (*ArdLedArrayAnimation)(ArdLedArray *, uint32_t, uint8_t);

    typedef struct ArdLed
    {
        /**
         * @brief   Array of all LEDs from pin
         *
         */
        CRGB *leds;

        /**
         * @brief   Number of LEDs
         *
         */
        size_t num_leds;

        /**
         * @brief   List of LED arrays
         *
         */
        ArdLedArray *arrays;
        /**
         * @brief   Number of LED arrays
         *
         */
        uint16_t num_arrays;

        /**
         * @brief   Update rate for animation
         *
         */
        uint32_t step_size;

        /**
         * @brief   Previous step time
         *
         */
        uint32_t prev_step;

    } ArdLed;

    // All LEDs
    //------------

    /**
     * @brief   Allocate LED data
     *
     * @param     led           DESCRIPTION
     * @param     num_arrays    DESCRIPTION
     * @param     num_leds_in_arrays   DESCRIPTION
     * @param     step_size     DESCRIPTION
     * @param     brightness    DESCRIPTION
     * @return    int           DESCRIPTION
     */
    int ard_led_alloc(ArdLed *led, const ArdLedParameters *parameters, const uint16_t *num_leds_in_arrays, uint16_t num_arrays);

    // int ard_led_wsalloc(ArdLed *led, const ArdLedParameters *parameters);

    /**
     * @brief   Free LED memory
     *
     * @param     led           LED container
     */
    void ard_led_free(ArdLed *led);

    /**
     * @brief   Adjust brightness
     *
     * @param     brightness    DESCRIPTION
     */
    void ard_led_set_brightness(const uint8_t brightness);

    /**
     * @brief   Initialize animations
     *
     * @param     led           DESCRIPTION
     */
    void ard_led_initialize(ArdLed *led);

    /**
     * @brief   Reset all led animations to start index
     *
     * @param     led           LED container
     */
    void ard_led_reset(ArdLed *led, const CRGB color, const uint32_t now);

    /**
     * @brief   Run all animations
     *
     * @param     led           LED container
     * @return    time          Time in microseconds
     * @return    bool          DESCRIPTION
     */
    bool ard_led_run(ArdLed *led, const uint32_t now);

    /**
     * @brief   Set color for all LEDs
     *
     * @param     led           LED container
     * @return    int           DESCRIPTION
     */
    void ard_led_set_color(ArdLed *led, const CRGB color);

    /**
     * @brief   Set all to white
     *
     * @return    int           DESCRIPTION
     */
    void ard_led_white(ArdLed *led);

    /**
     * @brief   Set all to black
     *
     * @return    int           DESCRIPTION
     */
    void ard_led_black(ArdLed *led);

    /**
     * @brief   Light up LEDs
     *
     * @return    int           DESCRIPTION
     */
    void ard_led_show();

    // Animations
    //-----------

    void ard_animation_rainbow(ArdLedArray *array, uint32_t step_size, uint8_t init);
    void ard_animation_twinkle(ArdLedArray *array, uint32_t step_size, uint8_t init);
    void ard_animation_powerup(ArdLedArray *array, uint32_t step_size, uint8_t init);
    void ard_animation_powerdown(ArdLedArray *array, uint32_t step_size, uint8_t init);

    // Animations Control
    //-------------------

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
    void ard_led_array_set_animation(ArdLed *led, const uint16_t array,
                                     ArdLedArrayAnimation animation, const CRGB color,
                                     const int16_t scale, const int16_t speed, const int16_t threshold);

    /**
     * @brief   Set array animation index
     *
     * @param     led           LED container
     * @param     array         Array index
     * @param     index         LED index in array
     * @return    int           DESCRIPTION
     */
    void ard_led_array_set_animation_index(ArdLed *led, const uint16_t array, const uint16_t index);
    /**
     * @brief   Set array animation scale
     *
     * @param     led           LED container
     * @param     array         Array index
     * @param     scale         Animation scale
     * @return    int           DESCRIPTION
     */
    void ard_led_array_set_animation_scale(ArdLed *led, const uint16_t array, const int16_t scale);
    /**
     * @brief   Set array color
     *
     * @param     led           LED container
     * @param     array         Array index
     * @param     color         Animation color
     * @return    int           DESCRIPTION
     */
    void ard_led_array_set_animation_color(ArdLed *led, const uint16_t array, const CRGB color);
    /**
     * @brief   Set array animation speed
     *
     * @param     led           LED container
     * @param     array         Array index
     * @param     speed         Animation Speed
     * @return    int           DESCRIPTION
     */
    void ard_led_array_set_animation_speed(ArdLed *led, const uint16_t array, const int16_t speed);
    /**
     * @brief   Set array animation threshold
     *
     * @param     led           LED container
     * @param     array         Array index
     * @param     threshold     Animation threshold
     * @return    int           DESCRIPTION
     */
    void ard_led_array_set_animation_threshold(ArdLed *led, const uint16_t array, const int16_t threshold);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
