/*
 * Simple 702 (Hakko 907) soldering station
 * ATmega8 (8 MHz internal)
 * and MAX6675 K-Thermocouple to Digital Converter
 *
 * Use [MiniCore](https://github.com/MCUdude/MiniCore)
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// Configuration
#define DEBUG

// Temperature limits
#define MIN_TEMPERATURE     180
#define MAX_TEMPERATURE     450
#define DEFAULT_TEMPERATURE 250
#define TEMP_HYSTERESIS     10

// Timing constants
#define BUTTON_DEBOUNCE_MS     150
#define SENSOR_READ_INTERVAL_MS 250
#define DISPLAY_TIMEOUT_MS     2000
#define SERIAL_DEBUG_INTERVAL_MS 1000

// Pin definitions
#define BTN_LEFT_PIN   PC3  // PC3 (ADC3) A3
#define BTN_RIGHT_PIN  PC2  // PC2 (ADC2) A2
#define HEATER_PIN     PB1  // D9
#define GREEN_LED_PIN  PC5  // PC5 (ADC5/SCL)
#define RED_LED_PIN    PC4  // PC4 (ADC4/SDA)

// MAX6675 pins
#define MISO_PIN PD2
#define CS_PIN   PD3
#define SCLK_PIN PD4

// 7-segment display segment pins
#define PIN_7SEG_A  PD6  // 6
#define PIN_7SEG_B  PB0  // 8
#define PIN_7SEG_C  PB3  // 11
#define PIN_7SEG_D  PC0  // A0
#define PIN_7SEG_E  PB4  // 12
#define PIN_7SEG_F  PD7  // 7
#define PIN_7SEG_G  PB2  // 10
#define PIN_7SEG_DP PB5  // 13

// 7-segment display digit select pins
#define PIN_7SEG_ACT_LEFT   PD5  // D5
#define PIN_7SEG_ACT_MIDDLE PB7  // PORTB7 (XTAL2)
#define PIN_7SEG_ACT_RIGHT  PB6  // PORTB6 (XTAL1)

// Bit manipulation macros
#define SET_BIT(port, pin)    ((port) |= (1 << (pin)))
#define CLEAR_BIT(port, pin)  ((port) &= ~(1 << (pin)))
#define TOGGLE_BIT(port, pin) ((port) ^= (1 << (pin)))
#define READ_BIT(pinr, pin)   (((pinr) >> (pin)) & 0x01)

// Hardware control macros
#define HEATER_ON()     SET_BIT(PORTB, HEATER_PIN)
#define HEATER_OFF()    CLEAR_BIT(PORTB, HEATER_PIN)

#define GREEN_LED_ON()  SET_BIT(PORTC, GREEN_LED_PIN)
#define GREEN_LED_OFF() CLEAR_BIT(PORTC, GREEN_LED_PIN)

#define RED_LED_ON()    SET_BIT(PORTC, RED_LED_PIN)
#define RED_LED_OFF()   CLEAR_BIT(PORTC, RED_LED_PIN)

// Global variables
static float temperature_correction = 0.75f;
static float current_temperature = 0.0f;
static float target_temperature = DEFAULT_TEMPERATURE;

static uint8_t last_button_state = 0;
static int16_t display_value = 0;
static int16_t display_active_value = 0;

static uint32_t last_key_press_time = 0;
static uint32_t last_sensor_read_time = 0;
static uint32_t last_serial_debug_time = 0;

static bool display_enabled = true;
static bool display_blink = false;

static uint8_t current_segment = 0;
static uint8_t current_digit = 0;

// Button state enumeration
typedef enum {
    BUTTON_NONE = 0,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_BOTH
} button_state_t;

// 7-segment display patterns for digits 0-9
// Bits: A B C D E F G (DP not used in patterns)
static const uint8_t digit_patterns[10] = {
    0b1111110,  // 0
    0b0110000,  // 1
    0b1101101,  // 2
    0b1111001,  // 3
    0b0110011,  // 4
    0b1011011,  // 5
    0b1011111,  // 6
    0b1110000,  // 7
    0b1111111,  // 8
    0b1111011   // 9
};

/**
 * @brief Select active digit on 7-segment display
 * @param digit_num Digit number: 0-left, 1-middle, 2-right, -1 to turn off all
 */
static void select_7seg_digit(int8_t digit_num) {
    switch (digit_num) {
        case 0:  // Left digit
            CLEAR_BIT(PORTD, PIN_7SEG_ACT_LEFT);
            SET_BIT(PORTB, PIN_7SEG_ACT_RIGHT);
            CLEAR_BIT(PORTB, PIN_7SEG_ACT_MIDDLE);
            break;
            
        case 1:  // Middle digit
            CLEAR_BIT(PORTD, PIN_7SEG_ACT_LEFT);
            CLEAR_BIT(PORTB, PIN_7SEG_ACT_RIGHT);
            SET_BIT(PORTB, PIN_7SEG_ACT_MIDDLE);
            break;
            
        case 2:  // Right digit
            SET_BIT(PORTD, PIN_7SEG_ACT_LEFT);
            CLEAR_BIT(PORTB, PIN_7SEG_ACT_RIGHT);
            CLEAR_BIT(PORTB, PIN_7SEG_ACT_MIDDLE);
            break;
            
        default:  // Turn off all digits
            CLEAR_BIT(PORTD, PIN_7SEG_ACT_LEFT);
            CLEAR_BIT(PORTB, PIN_7SEG_ACT_RIGHT);
            CLEAR_BIT(PORTB, PIN_7SEG_ACT_MIDDLE);
            break;
    }
}

/**
 * @brief Display a digit on the 7-segment display
 * @param digit Digit to display (0-9)
 */
static void set_7seg_digit(uint8_t digit) {
    if (!display_enabled) {
        // Turn off all segments
        CLEAR_BIT(PORTD, PIN_7SEG_A);
        CLEAR_BIT(PORTB, PIN_7SEG_B);
        CLEAR_BIT(PORTB, PIN_7SEG_C);
        CLEAR_BIT(PORTC, PIN_7SEG_D);
        CLEAR_BIT(PORTB, PIN_7SEG_E);
        CLEAR_BIT(PORTD, PIN_7SEG_F);
        CLEAR_BIT(PORTB, PIN_7SEG_G);
        return;
    }
    
    if (digit > 9) {
        digit = 0;  // Safety check
    }
    
    uint8_t pattern = digit_patterns[digit];
    
    // Set segment A
    if (pattern & 0b1000000) SET_BIT(PORTD, PIN_7SEG_A);
    else CLEAR_BIT(PORTD, PIN_7SEG_A);
    
    // Set segment B
    if (pattern & 0b0100000) SET_BIT(PORTB, PIN_7SEG_B);
    else CLEAR_BIT(PORTB, PIN_7SEG_B);
    
    // Set segment C
    if (pattern & 0b0010000) SET_BIT(PORTB, PIN_7SEG_C);
    else CLEAR_BIT(PORTB, PIN_7SEG_C);
    
    // Set segment D
    if (pattern & 0b0001000) SET_BIT(PORTC, PIN_7SEG_D);
    else CLEAR_BIT(PORTC, PIN_7SEG_D);
    
    // Set segment E
    if (pattern & 0b0000100) SET_BIT(PORTB, PIN_7SEG_E);
    else CLEAR_BIT(PORTB, PIN_7SEG_E);
    
    // Set segment F
    if (pattern & 0b0000010) SET_BIT(PORTD, PIN_7SEG_F);
    else CLEAR_BIT(PORTD, PIN_7SEG_F);
    
    // Set segment G
    if (pattern & 0b0000001) SET_BIT(PORTB, PIN_7SEG_G);
    else CLEAR_BIT(PORTB, PIN_7SEG_G);
}

/**
 * @brief Timer interrupt handler for display multiplexing
 */
static void timer_display_multiplex(void) {
    current_segment = (current_segment >= 2) ? 0 : current_segment + 1;
    
    switch (current_segment) {
        case 0:  // Units
            display_active_value = display_value;
            current_digit = display_active_value % 10;
            break;
            
        case 1:  // Tens
            current_digit = (display_active_value / 10) % 10;
            break;
            
        case 2:  // Hundreds
            current_digit = display_active_value / 100;
            break;
    }
    
    // Turn off display, set segments, then enable digit
    select_7seg_digit(-1);
    set_7seg_digit(current_digit);
    select_7seg_digit(current_segment);
}

/**
 * @brief Initialize timer for display multiplexing
 */
static void init_timer(void) {
    // Enable Timer2 overflow interrupt
    TIMSK |= (1 << TOIE2);
    // Set prescaler to 8 (if needed, currently commented out)
    // TCCR2 |= (1 << CS21);
    
    // Enable global interrupts
    sei();
}

/**
 * @brief Read and debounce button presses
 * @return Button state (BUTTON_NONE, BUTTON_LEFT, BUTTON_RIGHT, BUTTON_BOTH)
 */
static button_state_t read_buttons(void) {
    static button_state_t last_button = BUTTON_NONE;
    button_state_t current_state = BUTTON_NONE;
    
    // Read button states (active HIGH based on original code)
    bool left_pressed = (PINC & (1 << BTN_LEFT_PIN)) != 0;
    bool right_pressed = (PINC & (1 << BTN_RIGHT_PIN)) != 0;
    
    // Determine current state
    if (left_pressed && right_pressed) {
        current_state = BUTTON_BOTH;
    } else if (left_pressed) {
        current_state = BUTTON_LEFT;
    } else if (right_pressed) {
        current_state = BUTTON_RIGHT;
    } else {
        current_state = BUTTON_NONE;
    }
    
    // Edge detection: return button only on rising edge
    button_state_t result = BUTTON_NONE;
    if ((current_state != last_button) && (current_state != BUTTON_NONE)) {
        result = current_state;
    }
    
    last_button = current_state;
    return result;
}

/**
 * @brief Read temperature from MAX6675
 * @return Temperature in Celsius, or -1 if error
 */
static float read_temperature_celsius(void) {
    uint16_t raw_data = 0;
    
    // Start conversion
    CLEAR_BIT(PORTD, CS_PIN);
    _delay_ms(1);
    
    // Read 16 bits
    raw_data = spi_read_byte();
    raw_data <<= 8;
    raw_data |= spi_read_byte();
    
    // End conversion
    SET_BIT(PORTD, CS_PIN);
    
    // Check for thermocouple error
    if (raw_data & 0x04) {
        #ifdef DEBUG
        Serial.println("ERROR: No thermocouple attached!");
        #endif
        return -1.0f;
    }
    
    // Convert to temperature (12-bit resolution, LSB = 0.25°C)
    raw_data >>= 3;
    return raw_data * 0.25f;
}

/**
 * @brief Read one byte from SPI
 * @return Byte read from SPI
 */
static uint8_t spi_read_byte(void) {
    uint8_t data = 0;
    
    for (int8_t i = 7; i >= 0; i--) {
        CLEAR_BIT(PORTD, SCLK_PIN);
        _delay_ms(1);
        
        if (PIND & (1 << MISO_PIN)) {
            data |= (1 << i);
        }
        
        SET_BIT(PORTD, SCLK_PIN);
        _delay_ms(1);
    }
    
    return data;
}

/**
 * @brief Setup function - runs once at startup
 */
void setup(void) {
    #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Initializing Hakko 907 Soldering Station...");
    #endif
    
    // Configure 7-segment display segment pins as outputs
    DDRB |= (1 << PIN_7SEG_B) | (1 << PIN_7SEG_C) | 
            (1 << PIN_7SEG_E) | (1 << PIN_7SEG_G) | 
            (1 << PIN_7SEG_DP);
    DDRD |= (1 << PIN_7SEG_A) | (1 << PIN_7SEG_F);
    DDRC |= (1 << PIN_7SEG_D);
    
    // Configure heater pin as output
    DDRB |= (1 << HEATER_PIN);
    
    // Configure digit select pins as outputs
    DDRB |= (1 << PIN_7SEG_ACT_MIDDLE) | (1 << PIN_7SEG_ACT_RIGHT);
    DDRD |= (1 << PIN_7SEG_ACT_LEFT);
    
    // Configure button pins as inputs (with internal pull-ups disabled)
    DDRC &= ~((1 << BTN_LEFT_PIN) | (1 << BTN_RIGHT_PIN));
    PORTC &= ~((1 << BTN_LEFT_PIN) | (1 << BTN_RIGHT_PIN));
    
    // Configure LED pins as outputs
    DDRC |= (1 << GREEN_LED_PIN) | (1 << RED_LED_PIN);
    
    // Configure MAX6675 SPI pins
    DDRD |= (1 << CS_PIN) | (1 << SCLK_PIN);  // Outputs
    DDRD &= ~(1 << MISO_PIN);                 // Input
    SET_BIT(PORTD, CS_PIN);                   // CS high (inactive)
    
    // Turn off all display digits initially
    select_7seg_digit(-1);
    
    // Initialize timer for display multiplexing
    init_timer();
    
    #ifdef DEBUG
    Serial.println("Initialization complete.");
    #endif
}

/**
 * @brief Main loop - runs continuously
 */
void loop(void) {
    uint32_t current_time = millis();
    
    // Handle button presses with debouncing
    if (current_time - last_key_press_time > BUTTON_DEBOUNCE_MS) {
        button_state_t button = read_buttons();
        
        switch (button) {
            case BUTTON_LEFT:
                if (target_temperature > MIN_TEMPERATURE) {
                    target_temperature -= 1.0f;
                    last_key_press_time = current_time;
                }
                break;
                
            case BUTTON_RIGHT:
                if (target_temperature < MAX_TEMPERATURE) {
                    target_temperature += 1.0f;
                    last_key_press_time = current_time;
                }
                break;
                
            case BUTTON_BOTH:
                target_temperature = DEFAULT_TEMPERATURE;
                last_key_press_time = current_time;
                break;
                
            case BUTTON_NONE:
                // No button pressed
                break;
        }
    }
    
    // Read temperature sensors periodically
    if (current_time - last_sensor_read_time > SENSOR_READ_INTERVAL_MS) {
        last_sensor_read_time = current_time;
        
        // Read current temperature with correction
        float raw_temp = read_temperature_celsius();
        if (raw_temp >= 0) {
            current_temperature = raw_temp * temperature_correction;
        } else {
            current_temperature = -1.0f;
        }
        
        // Update display value (show target temp for 2 seconds after button press)
        if (current_time - last_key_press_time < DISPLAY_TIMEOUT_MS) {
            display_value = (int16_t)target_temperature;
        } else {
            display_value = (int16_t)current_temperature;
        }
        
        // Control heater and LEDs based on temperature
        if (current_temperature < 0) {
            // Error condition - no thermocouple
            HEATER_OFF();
            RED_LED_OFF();
            GREEN_LED_OFF();
            display_enabled = false;  // Optional: blink display on error
            
        } else if (current_temperature < (target_temperature - TEMP_HYSTERESIS)) {
            // Heat up - far below target
            HEATER_ON();
            RED_LED_ON();
            GREEN_LED_OFF();
            display_enabled = true;
            
        } else if (current_temperature < target_temperature) {
            // Approaching target - reduce heating
            HEATER_ON();
            RED_LED_OFF();
            GREEN_LED_ON();
            display_enabled = true;
            
        } else {
            // At or above target - maintain temperature
            HEATER_OFF();
            RED_LED_OFF();
            GREEN_LED_ON();
            display_enabled = true;
        }
    }
    
    // Debug output (if enabled)
    #ifdef DEBUG
    if (current_time - last_serial_debug_time > SERIAL_DEBUG_INTERVAL_MS) {
        last_serial_debug_time = current_time;
        
        Serial.print("Current: ");
        Serial.print(current_temperature);
        Serial.print("°C, Target: ");
        Serial.print(target_temperature);
        Serial.print("°C, Heater: ");
        Serial.println((PORTB & (1 << HEATER_PIN)) ? "ON" : "OFF");
    }
    #endif
}

/**
 * @brief Timer2 overflow interrupt service routine
 */
ISR(TIMER2_OVF_vect) {
    timer_display_multiplex();
}
