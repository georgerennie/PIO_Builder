// pio_blinky example using pio_builder
// https://github.com/raspberrypi/pico-examples/blob/master/pio/pio_blinky

// Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
// SPDX-License-Identifier: BSD-3-Clause

#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pio_builder.hpp"

static struct PIOBlink : public PIOBuilder::Program<> {
	PIOBlink() {
		// clang-format off
			pull_block();
			out_y(32);
		wrap_target();
			mov(MovDest::X, MovSrc::Y);
			set_pins(1);         // Turn LED on

		auto lp1 = label();
			jmp_x_dec(lp1);      // Delay for (x + 1) cycles, x is a 32 bit number
			mov(MovDest::X, MovSrc::Y);
			set_pins(0);         // Turn LED off

		auto lp2 = label();
			jmp_x_dec(lp2);      // Delay for the same number of cycles again
		wrap();                  // Blink forever!
		// clang-format on
	}
} blink_program;

// this is a raw helper function for use by the user which sets up the GPIO output,
// and configures the SM to output on a particular pin
void blink_program_init(PIO pio, uint sm, uint offset, uint pin) {
	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
	pio_sm_config c = blink_program.get_default_config(offset);
	sm_config_set_set_pins(&c, pin, 1);
	pio_sm_init(pio, sm, offset, &c);
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

int main() {
	setup_default_uart();

	// todo get free sm
	PIO pio     = pio0;
	uint offset = pio_add_program(pio, blink_program);
	printf("Loaded program at %d\n", offset);

#ifdef PICO_DEFAULT_LED_PIN
	blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
#else
	blink_pin_forever(pio, 0, offset, 0, 3);
#endif
	blink_pin_forever(pio, 1, offset, 6, 4);
	blink_pin_forever(pio, 2, offset, 11, 1);
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
	blink_program_init(pio, sm, offset, pin);
	pio_sm_set_enabled(pio, sm, true);

	printf("Blinking pin %d at %d Hz\n", pin, freq);

	// PIO counter program takes 3 more cycles in total than we pass as
	// input (wait for n + 1; mov; jmp)
	pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
