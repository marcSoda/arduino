#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdint.h>

/* ---------------- Pins ---------------- */

#define UPS_PIN     PB1   // UPS SW+ control (via transistor)
#define REQ_PIN     PB0   // Pi shutdown request (GPIO13 via transistor)
#define SW_PIN      PB3   // Toggle switch to GND (ON = LOW)
#define DONE_PIN    PB4   // gpio-poweroff signal (Pi GPIO26) + 100k pulldown to GND
#define LED_PIN     PB2   // Status LED (series resistor to GND)

/* ---------------- Timing ---------------- */

#define SHUTDOWN_TIMEOUT_MS  10000
#define POLL_MS              10

/* ---------------- Interrupt wake flag ---------------- */

static volatile uint8_t woke = 0;

ISR(PCINT0_vect) {
    woke = 1;
}

/* ---------------- Helpers ---------------- */

static inline uint8_t switch_on(void) {
    return !(PINB & (1 << SW_PIN));      // active-low
}

static inline uint8_t done_high(void) {
    return  (PINB & (1 << DONE_PIN));    // HIGH
}

static inline uint8_t done_low(void) {
    return !(PINB & (1 << DONE_PIN));    // LOW
}

static inline void ups_on(void)  { PORTB |=  (1 << UPS_PIN); }
static inline void ups_off(void) { PORTB &= ~(1 << UPS_PIN); }

static inline void led_on(void)  { PORTB |=  (1 << LED_PIN); }
static inline void led_off(void) { PORTB &= ~(1 << LED_PIN); }

static void blink(uint8_t count, uint16_t on_ms, uint16_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        led_on();  _delay_ms(on_ms);
        led_off(); _delay_ms(off_ms);
    }
}

static void request_shutdown(void) {
    PORTB |=  (1 << REQ_PIN);
    _delay_ms(200);
    PORTB &= ~(1 << REQ_PIN);
}

/*
 * Sleep until a pin-change occurs on SW_PIN or DONE_PIN.
 * Uses POWER-DOWN for low idle current.
 */
static void sleep_until_pin_change(void) {
    woke = 0;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();          // wakes on PCINT
    sleep_disable();
}

/* ---------------- Init ---------------- */

static void io_init(void) {
    /* Outputs */
    DDRB |= (1 << UPS_PIN) | (1 << REQ_PIN) | (1 << LED_PIN);
    ups_off();
    led_off();
    PORTB &= ~(1 << REQ_PIN);

    /* Inputs */
    DDRB &= ~((1 << SW_PIN) | (1 << DONE_PIN));
    PORTB |= (1 << SW_PIN);    // pull-up on switch
    /* DONE_PIN has external pulldown; no internal pull-up. */
}

static void pcint_init(void) {
    /* Enable pin-change interrupts on PB2 and PB3 */
    GIMSK |= (1 << PCIE);
    PCMSK |= (1 << PCINT3) | (1 << PCINT4);
    sei();
}

/* ---------------- Main ---------------- */

int main(void) {
    io_init();
    pcint_init();

    while (1) {

        /* ---- WAIT FOR SWITCH ON (sleeping) ---- */
        while (!switch_on()) {
            sleep_until_pin_change();
            _delay_ms(10); // debounce
        }

        /* ---- POWER ON ---- */
        ups_on();

        /*
         * Critical rule:
         * Only treat DONE_PIN going LOW as "shutdown complete" if we have first seen it HIGH
         * in this power cycle. This prevents false graceful detection at boot / with broken wiring.
         */
        uint8_t seen_running = 0;

        /* ---- WAIT FOR SWITCH OFF (sleeping) ---- */
        while (switch_on()) {
            if (done_high()) {
                seen_running = 1;
            }
            sleep_until_pin_change();
            _delay_ms(10); // debounce
        }

        /* ---- SHUTDOWN REQUEST ---- */
        request_shutdown();

        /* ---- WAIT FOR gpio-poweroff OR TIMEOUT ---- */
        uint16_t elapsed = 0;
        uint8_t graceful = 0;

        while (elapsed < SHUTDOWN_TIMEOUT_MS) {

            /* Track that Pi has been running at least once */
            if (done_high()) {
                seen_running = 1;
            }

            /* Graceful only valid if Pi was seen running */
            if (seen_running && done_low()) {
                graceful = 1;
                break;
            }

            _delay_ms(POLL_MS);
            elapsed += POLL_MS;
        }

        /* ---- USER FEEDBACK ---- */
        if (graceful) {
            blink(5, 500, 500);   // 5 blinks, 1 Hz
        } else {
            blink(20, 50, 50);    // 20 blinks in 2 seconds
        }

        /* ---- CUT POWER ---- */
        ups_off();
        _delay_ms(500);

        /*
         * If the user flipped the switch back ON during shutdown,
         * immediately re-apply power after the power cut (power-cycle boot).
         */
        if (switch_on()) {
            ups_on();
        }
    }
}
