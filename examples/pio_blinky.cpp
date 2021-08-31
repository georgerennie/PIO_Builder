// A recreation of pio_blink using pio_builder
// https://github.com/raspberrypi/pico-examples/blob/master/pio/pio_blink/blink.c

#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pio_builder.hpp"

using namespace PIOBuilder;

Program<> blink_program;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void configure_program();

int main() {
	setup_default_uart();

	configure_program();

	// todo get free sm
	PIO pio = pio0;
	uint offset = pio_add_program(pio, blink_program);
	printf("Loaded program at %d\n", offset);

	blink_pin_forever(pio, 0, offset, 0, 3);
	blink_pin_forever(pio, 1, offset, 6, 4);
	blink_pin_forever(pio, 2, offset, 11, 1);
}

void configure_program() {
	auto &p = blink_program;

	p.pull(true);          // pull block
	p.out(OutDest::Y, 32); // out y, 32

	p.wrap_target();              // .wrap_target
	p.mov(MovDest::X, MovSrc::Y); // mov x, y
	p.set(SetDest::Pins, 0);      // set pins, 0

	const auto lp1 = p.jmp(JmpCond::X_PostDec); // jmp x--
	p.set_jmp_dest(lp1, lp1);                   // lp1 jumps to itself
	p.mov(MovDest::X, MovSrc::Y);
	p.set(SetDest::Pins, 1); // set pins, 1

	const auto lp2 = p.jmp(JmpCond::X_PostDec); // jmp x--
	p.set_jmp_dest(lp2, lp2);                   // lp2 jumps to itself
	p.wrap();                                   // .wrap
}

void blink_program_init(PIO pio, uint sm, uint offset, uint pin) {
	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
	pio_sm_config c = blink_program.get_default_config(offset);
	sm_config_set_set_pins(&c, pin, 1);
	pio_sm_init(pio, sm, offset, &c);
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
	blink_program_init(pio, sm, offset, pin);
	pio_sm_set_enabled(pio, sm, true);

	printf("Blinking pin %d at %d Hz\n", pin, freq);
	pio->txf[sm] = clock_get_hz(clk_sys) / 2 * freq;
}
