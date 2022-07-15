// i2c example. This is not a full program, just a demonstration of how a more
// complex pio program can be written with PIOBuilder
// https://github.com/raspberrypi/pico-examples/blob/master/pio/i2c/i2c.pio

// Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
// SPDX-License-Identifier: BSD-3-Clause

#include "pio_builder.hpp"

// TX Encoding:
// | 15:10 | 9     | 8:1  | 0   |
// | Instr | Final | Data | NAK |
//
// If Instr has a value n > 0, then this FIFO word has no
// data payload, and the next n + 1 words will be executed as instructions.
// Otherwise, shift out the 8 data bits, followed by the ACK bit.
//
// The Instr mechanism allows stop/start/repstart sequences to be programmed
// by the processor, and then carried out by the state machine at defined points
// in the datastream.
//
// The "Final" field should be set for the final byte in a transfer.
// This tells the state machine to ignore a NAK: if this field is not
// set, then any NAK will cause the state machine to halt and interrupt.
//
// Autopull should be enabled, with a threshold of 16.
// Autopush should be enabled, with a threshold of 8.
// The TX FIFO should be accessed with halfword writes, to ensure
// the data is immediately available in the OSR.
//
// Pin mapping:
// - Input pin 0 is SDA, 1 is SCL (if clock stretching used)
// - Jump pin is SDA
// - Side-set pin 0 is SCL
// - Set pin 0 is SDA
// - OUT pin 0 is SDA
// - SCL must be SDA + 1 (for wait mapping)
//
// The OE outputs should be inverted in the system IO controls!
// (It's possible for the inversion to be done in this program,
// but costs 2 instructions: 1 for inversion, and one to cope
// with the side effect of the MOV on TX shift counter.)

class I2C : Program<1, true, true> {
public:
	__attribute__((optimize(3), always_inline)) constexpr I2C() {
		// clang-format off
		auto do_nack = label();
			auto entry_point = jmp_fwd_y_dec();  // Continue if NAK was expected
			irq_wait_rel(0);                     // Otherwise stop, ask for help

		auto do_byte = label();
			set_x(7);                            // Loop 8 times
		auto bitloop = label();
			out_pindirs(1)                [7];   // Serialise write data (all-ones if reading)
			nop()                .side(1) [2];   // SCL rising edge
			wait_pin(1, 1)                [4];   // Allow clock to be stretched
			in_pins(1)                    [7];   // Sample read data in middle of SCL pulse
			jmp_x_dec(bitloop)   .side(0) [7];   // SCL falling edge

			// Handle ACK pulse
			out_pindirs(1)                [7];   // On reads, we provide the ACK.
			nop()                .side(1) [7];   // SCL rising edge
			wait_pin(1, 1)                [7];   // Allow clock to be stretched
			jmp_pin(do_nack)     .side(0) [2];   // Test SDA for ACK/NAK, fall through if ACK

		jmp_dest(entry_point);
		wrap_target();
			out_x(6);                            // Unpack Instr count
			out_y(1);                            // Unpack the NAK ignore bit
			jmp_not_x(do_byte);                  // Instr == 0, this is a data record.
			out_null(32);                        // Instr > 0, remainder of this OSR is invalid
		auto do_exec = label();
			out_exec(16);                        // Execute one instruction per FIFO word
			jmp_x_dec(do_exec);                  // Repeat n + 1 times
		wrap();
		// clang-format on
	}
};
