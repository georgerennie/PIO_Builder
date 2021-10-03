// SPDX-FileCopyrightText: 2021 George Rennie
// SPDX-License-Identifier: BSD-3-Clause

// This demo turns on and off pin 0 with the period being determined
// dynamically. The program could be repeatedly reconfigured with different
// values

#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pio_builder.hpp"

void configure_program(PIO pio, uint sm, uint pin, uint8_t output_period) {
	using namespace PIOBuilder;
	Program<2, 1> p; // 2 instructions, 1 sideset bit

	auto half_period = output_period / 2;

	p.wrap_target();
	p.nop(half_period, SideSet(1));
	p.nop(half_period, SideSet(0));
	p.wrap();

	uint offset = pio_add_program(pio, p);

	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
	auto config = p.get_default_config(offset);
	sm_config_set_sideset_pins(&config, pin);

	sm_config_set_clkdiv_int_frac(&config, ~0, ~0);

	pio_sm_init(pio, sm, offset, &config);
}

int main() {
	// todo get free sm
	PIO pio = pio0;
	configure_program(pio, 0, 0, 20);
	pio_sm_set_enabled(pio, 0, true);
}
