// SPDX-FileCopyrightText: 2022 George Rennie
// SPDX-License-Identifier: BSD-3-Clause

// This demo turns on and off the LED pin with the period being determined
// dynamically. The program could be repeatedly reconfigured with different
// values

#include <stdio.h>
#include "hardware/pio.h"
#include "pio_builder.hpp"

struct VariableBlink : public PIOBuilder::Program<1> {
	VariableBlink(uint8_t period) {
		const auto quarter_period = period / 4;

		// clang-format off
		wrap_target();
			nop()       .side(0) [quarter_period - 1];
			nop()       .side(0) [quarter_period - 1];
			nop()       .side(1) [quarter_period - 1];
			nop()       .side(1) [quarter_period - 1];
		wrap();
		// clang-format on
	}
};

void configure_program(PIO pio, uint sm, uint pin, uint8_t output_period) {
	VariableBlink program(output_period);

	uint offset = pio_add_program(pio, program);

	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
	auto config = program.get_default_config(offset);
	sm_config_set_sideset_pins(&config, pin);

	sm_config_set_clkdiv_int_frac(&config, ~0, ~0);
	pio_sm_init(pio, sm, offset, &config);
}

int main() {
	PIO pio = pio0;
	auto sm = pio_claim_unused_sm(pio, true);
	// Configure for 0.5s period (2Hz) clock
	configure_program(pio, sm, PICO_DEFAULT_LED_PIN, 60);
	pio_sm_set_enabled(pio, sm, true);
}
