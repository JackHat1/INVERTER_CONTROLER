#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"


// Only the timer flag must be global for the callback
volatile bool sine_update_flag = false; // Timer update flag

// Function declarations
void uart_thread(); // UART command handler thread
void setupPWM(uint gpio_pin); // Setup PWM on a pin
void set_pwm_frequency(uint gpio_pin, float pwm_freq); // Set PWM frequency
void print_help(); // Print help message
void check_uart_command(float* amp, float* stepSize, float* pwm_freq, bool* led_state); // UART command handler
bool sine_timer_callback(repeating_timer_t *rt); // Timer callback

    #define SINE_STEPS         360         // Sine table steps
    #define DEAD_TIME_NS       300         // Dead time (not used)
    #define PWM_RESOLUTION     255         // PWM resolution
    #define UPDATE_RATE_HZ     100000.0f   // Sine update rate
    #define FIXED_FREQ_HZ      500.0f      // Fixed sine frequency
    #define START_AMPLITUDE    1.0f        // Default amplitude
    
    volatile float amp       = START_AMPLITUDE;     // Sine amplitude
    volatile float stepSize  = 0.0f;                // Sine step per update
    volatile float sineIndex = 0.0f;                // Current sine index
    volatile float pwm_freq  = 20000.0f;            // PWM carrier frequency (Hz)
    volatile bool led_state  = false;               // LED state
int main()
{   
    // GPIO Pin Constants /////////////////////////////////////////////////////
    const uint PIN_LED   = 25;   // Onboard LED
    const uint PWM_PIN_A = 2;    // PWM output A
    const uint PWM_PIN_B = 3;    // PWM output B
    ///////////////////////////////////////////////////////////////////////////

    // // Waveform Configuration Constants ///////////////////////////////////////
    // #define SINE_STEPS         360         // Sine table steps
    // #define DEAD_TIME_NS       300         // Dead time (not used)
    // #define PWM_RESOLUTION     255         // PWM resolution
    // #define UPDATE_RATE_HZ     100000.0f   // Sine update rate
    // #define FIXED_FREQ_HZ      500.0f      // Fixed sine frequency
    // #define START_AMPLITUDE    1.0f        // Default amplitude
    // ///////////////////////////////////////////////////////////////////////////

    // // User Variables /////////////////////////////////////////////////////////
    // volatile float amp       = START_AMPLITUDE;     // Sine amplitude
    // volatile float stepSize  = 0.0f;                // Sine step per update
    // volatile float sineIndex = 0.0f;                // Current sine index
    // volatile float pwm_freq  = 20000.0f;            // PWM carrier frequency (Hz)
    // volatile bool led_state  = false;               // LED state
    // ///////////////////////////////////////////////////////////////////////////

    // GPIO Initialization ////////////////////////////////////////////////////
    gpio_init(PIN_LED);                          // Init onboard LED
    gpio_init(PWM_PIN_A);                        // Init PWM pin A
    gpio_init(PWM_PIN_B);                        // Init PWM pin B
    gpio_set_dir(PIN_LED, GPIO_OUT);             // Set LED as output
    ///////////////////////////////////////////////////////////////////////////

    // Serial and PWM Setup ///////////////////////////////////////////////////
    stdio_init_all();                            // Init USB/serial
    setupPWM(PWM_PIN_A);                         // Setup PWM on pin A
    setupPWM(PWM_PIN_B);                         // Setup PWM on pin B
    set_pwm_frequency(PWM_PIN_A, pwm_freq);      // Set PWM freq on pin A
    set_pwm_frequency(PWM_PIN_B, pwm_freq);      // Set PWM freq on pin B
    ///////////////////////////////////////////////////////////////////////////

    // Sine Step Calculation //////////////////////////////////////////////////
    stepSize = (FIXED_FREQ_HZ * SINE_STEPS) / UPDATE_RATE_HZ; // 500Hz sine
    ///////////////////////////////////////////////////////////////////////////

    // Startup Message ////////////////////////////////////////////////////////
    sleep_ms(2000);                              // Wait for USB
    printf("SPWM Inverter Ready.\n");            // Print ready
    print_help();                                // Print help
    ///////////////////////////////////////////////////////////////////////////

    // Timer Setup ////////////////////////////////////////////////////////////
    repeating_timer_t timer;                                                 // Timer variable
    float interval_us = (1.0f / UPDATE_RATE_HZ) * 1e6f;                      // Timer interval
    add_repeating_timer_us(-interval_us, sine_timer_callback, NULL, &timer); // Start timer
    ///////////////////////////////////////////////////////////////////////////
   
    // Launch UART handler on core 1///////////////////////////////////////////
    multicore_launch_core1(uart_thread);

    uint slice = pwm_gpio_to_slice_num(PWM_PIN_A);
    pwm_set_output_polarity(slice, false, true); // A not inverted, B inverted

    // Main Loop //////////////////////////////////////////////////////////////
    while (1) {
        if (sine_update_flag) {
            sine_update_flag = false;
            sineIndex += stepSize;
            if (sineIndex >= SINE_STEPS) sineIndex -= SINE_STEPS;
            float angle = sineIndex * 2.0f * M_PI / SINE_STEPS;
            float sine_val = amp * sinf(angle); // Range: -amp to +amp

            // Map sine_val from [-amp, +amp] to [0, PWM_RESOLUTION]
            uint8_t pwm_val = (uint8_t)(((sine_val + amp) / (2.0f * amp)) * PWM_RESOLUTION);

            pwm_set_gpio_level(PWM_PIN_A, pwm_val);
            pwm_set_gpio_level(PWM_PIN_B, pwm_val); // Both get the same value, B is inverted by hardware
        }
    }
    return 0;
}// End main()

// --- Helper Functions ---////////////////////////////////////////////////////
// Move your UART handler to a function for the second core
void uart_thread() {
    while (1) {
        check_uart_command(&amp, &stepSize, &pwm_freq, &led_state);
        sleep_ms(1); // Small delay to yield CPU
    }
}
bool sine_timer_callback(repeating_timer_t *rt) {
    sine_update_flag = true; // Set update flag
    return true;
} ///////////////////////////////////////////////////////////////////////////////

void setupPWM(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);           // Set the pin to PWM mode
    uint slice = pwm_gpio_to_slice_num(gpio_pin);         // Find which PWM slice controls this pin
    pwm_set_output_polarity(slice, false, true); // A not inverted, B inverted
    pwm_config config = pwm_get_default_config();         // Start with the default PWM config
    pwm_config_set_clkdiv(&config, 1.0f);                 // No clock division, run at full speed
    pwm_config_set_wrap(&config, PWM_RESOLUTION);         // Set the PWM resolution (counter top)
    pwm_init(slice, &config, true);                       // Initialize and start the PWM
} ///////////////////////////////////////////////////////////////////////////////

void set_pwm_frequency(uint gpio_pin, float pwm_freq) {
    uint slice = pwm_gpio_to_slice_num(gpio_pin);         // Get the PWM slice for this pin
    pwm_config config = pwm_get_default_config();         // Start with the default PWM config
    float clkdiv = (float)clock_get_hz(clk_sys)           // Calculate clock divider for desired frequency
                   / ((PWM_RESOLUTION + 1) * pwm_freq);
    pwm_config_set_clkdiv(&config, clkdiv);               // Set the calculated clock divider
    pwm_config_set_wrap(&config, PWM_RESOLUTION);         // Set the PWM resolution (counter top)
    pwm_init(slice, &config, true);                       // Initialize and start the PWM with new settings

    // Re-apply output polarity after re-initializing
    pwm_set_output_polarity(slice, false, true); // A normal, B inverted
}
///////////////////////////////////////////////////////////////////////////////

void print_help() {
    printf("Commands:\n"); // Print available commands
    printf("  a<amp>      Set amplitude (0.0 - 1.0)\n");
    printf("  q<freq>     Set PWM carrier frequency (Hz)\n");
    printf("  l           Toggle onboard LED\n");
    printf("  h           Print this help message\n");
    printf("  f<freq>     Set sine output frequency (Hz)\n");
    printf("Sine output is fixed at 500Hz.\n");
} ///////////////////////////////////////////////////////////////////////////////

void check_uart_command(float* amp, float* stepSize, float* pwm_freq, bool* led_state) 
{
    static char cmd_buf[32];
    static uint i = 0;
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;
    if (c == '\n' || c == '\r') {
        cmd_buf[i] = 0;
            switch (cmd_buf[0]) {
                case 'a': {
                    float v = atof(&cmd_buf[1]);
                    if (v >= 0.0f && v <= 1.0f) *amp = v, printf("Amplitude set to %.2f\n", *amp);
                    else printf("Invalid amplitude: %s\n", cmd_buf);
                    break;
                }
                case 'q': {
                    float v = atof(&cmd_buf[1]);
                    if (v > 100.0f && v < 1000000.0f) {
                        *pwm_freq = v;
                        set_pwm_frequency(2, v);
                        set_pwm_frequency(3, v);
                        printf("PWM frequency set to %.2f Hz\n", v);
                    } else printf("Invalid PWM frequency: %s\n", cmd_buf);
                    break;
                }
                case 'l':
                    *led_state = !(*led_state);
                    gpio_put(25, *led_state);
                    printf("LED %s\n", *led_state ? "ON" : "OFF");
                    break;
                case 'h':
                    print_help();
                    break;
            case 'f': {
                float v = atof(&cmd_buf[1]);
                if (v > 0.0f && v < (UPDATE_RATE_HZ / 2.0f)) {
                    *stepSize = (v * SINE_STEPS) / UPDATE_RATE_HZ;
                    printf("Sine frequency set to %.2f Hz\n", v);
                } else {
                    printf("Invalid sine frequency: %s\n", cmd_buf);
                }
                break;
            }                       
                default:
                    if (cmd_buf[0]) printf("Unknown command: %s\n", cmd_buf);
            }
        i = 0;
    } else if (i < sizeof(cmd_buf) - 1 && c != PICO_ERROR_TIMEOUT) {
        cmd_buf[i++] = (char)c;
    }
}
///////////////////////////////////////////////////////////////////////////////