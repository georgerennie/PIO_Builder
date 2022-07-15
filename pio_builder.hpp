// SPDX-FileCopyrightText: 2022 George Rennie
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PIO_BUILDER_HPP
#define PIO_BUILDER_HPP

#include <inttypes.h>
#include <algorithm>
#include <array>
#include <type_traits>
#include "hardware/pio.h"

#ifdef PIO_BUILDER_NO_ASSERT
	#define PIO_ASSERT(COND) \
		{}
#else
	#include <assert.h>
	#define PIO_ASSERT(COND) assert(COND)
#endif

namespace PIOBuilder {

template <uint8_t SIDE_SET_PINS, bool SIDE_SET_OPT, bool SIDE_SET_PINDIRS>
class Program;

template <uint8_t SIDE_SET_PINS, bool SIDE_SET_OPT>
class InstrRef {
protected:
	static constexpr uint8_t side_set_mask  = (1 << SIDE_SET_PINS) - 1;
	static constexpr uint8_t side_set_shift = 13 - SIDE_SET_PINS - SIDE_SET_OPT;
	static constexpr uint8_t delay_bits     = 5 - SIDE_SET_PINS - SIDE_SET_OPT;
	static constexpr uint8_t delay_mask     = (1 << delay_bits) - 1;
	static constexpr uint8_t delay_shift    = 8;
	uint16_t& instr;

public:
	constexpr InstrRef(uint16_t& instr) : instr(instr){};

	__attribute__((always_inline)) constexpr InstrRef side(const uint8_t side_set) {
		// TODO: Work out how to do side_set enable
		PIO_ASSERT(side_set <= side_set_mask);

		if constexpr (SIDE_SET_OPT) {
			instr |= 1 << 12;
		}

		instr &= ~(side_set_mask << side_set_shift);
		instr |= side_set << side_set_shift;
		return *this;
	}

	__attribute__((always_inline)) constexpr InstrRef delay(const uint8_t delay) {
		PIO_ASSERT(delay <= delay_mask);
		instr &= ~(delay_mask << delay_shift);
		instr |= delay << delay_shift;
		return *this;
	}

	constexpr InstrRef operator[](const uint8_t delay) { return this->delay(delay); }
};

template <uint8_t SIDE_SET_PINS, bool SIDE_SET_OPT>
class JmpRef : public InstrRef<SIDE_SET_PINS, SIDE_SET_OPT> {
private:
	using InstrRefT = InstrRef<SIDE_SET_PINS, SIDE_SET_OPT>;
	template <uint8_t, bool, bool>
	friend class Program;

	__attribute__((always_inline)) constexpr JmpRef jump_to(const uint8_t addr) {
		PIO_ASSERT(addr <= 0x1F);
		this->instr &= ~0x1F;
		this->instr |= addr;
		return *this;
	}

public:
	constexpr JmpRef(InstrRefT&& instr) : InstrRefT(std::move(instr)) {}
};

template <uint8_t SIDE_SET_PINS = 0, bool SIDE_SET_OPT = false, bool SIDE_SET_PINDIRS = false>
class Program {
public:
	constexpr Program(){};

	constexpr void origin(const uint8_t origin) { program.origin = origin; }
	constexpr std::size_t size() const { return index; }

	constexpr operator const pio_program_t*() {
		PIO_ASSERT(index <= 32);
		program.length = index;
		return &program;
	}

	constexpr pio_sm_config get_default_config(const uint16_t offset) const {
		auto config = pio_get_default_sm_config();
		if (index > 0) {
			const auto wrap = std::min(static_cast<uint8_t>(index - 1), wrap_idx);
			sm_config_set_wrap(&config, offset + wrap_target_idx, offset + wrap);
		}
		sm_config_set_sideset(&config, SIDE_SET_PINS, SIDE_SET_OPT, SIDE_SET_PINDIRS);
		return config;
	}

	constexpr const uint16_t& operator[](std::size_t idx) const {
		PIO_ASSERT(idx < index);
		return instructions[idx];
	}

	constexpr uint16_t& operator[](std::size_t idx) {
		PIO_ASSERT(idx < index);
		return instructions[idx];
	}

protected:
	using InstrRefT = InstrRef<SIDE_SET_PINS, SIDE_SET_OPT>;
	using JmpRefT   = JmpRef<SIDE_SET_PINS, SIDE_SET_OPT>;

	enum class JmpCond : uint8_t {
		Always   = 0x00,
		Not_X    = 0x01,
		X_Dec    = 0x02,
		Not_Y    = 0x03,
		Y_Dec    = 0x04,
		X_NE_Y   = 0x05,
		Pin      = 0x06,
		Not_OSRE = 0x07,
	};

	enum class WaitSrc : uint8_t { GPIO = 0x00, Pin = 0x01, IRQ = 0x02 };

	enum class InSrc : uint8_t {
		Pins = 0x00,
		X    = 0x01,
		Y    = 0x02,
		Null = 0x03,
		ISR  = 0x06,
		OSR  = 0x07,
	};

	enum class OutDest : uint8_t {
		Pins    = 0x00,
		X       = 0x01,
		Y       = 0x02,
		Null    = 0x03,
		PinDirs = 0x04,
		PC      = 0x05,
		ISR     = 0x06,
		Exec    = 0x07,
	};

	enum class MovSrc : uint8_t {
		Pins   = 0x00,
		X      = 0x01,
		Y      = 0x02,
		Null   = 0x03,
		Status = 0x05,
		ISR    = 0x06,
		OSR    = 0x07
	};

	enum class MovOp : uint8_t { None = 0x00, Invert = 0x01, BitReverse = 0x02 };

	enum class MovDest : uint8_t {
		Pins = 0x00,
		X    = 0x01,
		Y    = 0x02,
		Exec = 0x04,
		PC   = 0x05,
		ISR  = 0x06,
		OSR  = 0x07,
	};

	enum class SetDest : uint8_t {
		Pins    = 0x00,
		X       = 0x01,
		Y       = 0x02,
		PinDirs = 0x04,
	};

	// ---- JMP ----
	using Label = uint8_t;
	constexpr Label label() { return index; }

	constexpr InstrRefT jmp(const JmpCond cond, const Label addr) {
		return get_instr(0x00, to_underlying(cond), static_cast<uint8_t>(addr));
	}

	constexpr InstrRefT jmp(const Label addr) { return jmp(JmpCond::Always, addr); }
	constexpr InstrRefT jmp_not_x(const Label addr) { return jmp(JmpCond::Not_X, addr); }
	constexpr InstrRefT jmp_x_dec(const Label addr) { return jmp(JmpCond::X_Dec, addr); }
	constexpr InstrRefT jmp_not_y(const Label addr) { return jmp(JmpCond::Not_Y, addr); }
	constexpr InstrRefT jmp_y_dec(const Label addr) { return jmp(JmpCond::Y_Dec, addr); }
	constexpr InstrRefT jmp_x_ne_y(const Label addr) { return jmp(JmpCond::X_NE_Y, addr); }
	constexpr InstrRefT jmp_pin(const Label addr) { return jmp(JmpCond::Pin, addr); }
	constexpr InstrRefT jmp_not_osre(const Label addr) { return jmp(JmpCond::Not_OSRE, addr); }

	constexpr JmpRefT jmp_fwd(const JmpCond cond) {
		return get_instr(0x00, to_underlying(cond), 0x00);
	}

	constexpr JmpRefT jmp_fwd() { return jmp_fwd(JmpCond::Always); }
	constexpr JmpRefT jmp_fwd_not_x() { return jmp_fwd(JmpCond::Not_X); }
	constexpr JmpRefT jmp_fwd_x_dec() { return jmp_fwd(JmpCond::X_Dec); }
	constexpr JmpRefT jmp_fwd_not_y() { return jmp_fwd(JmpCond::Not_Y); }
	constexpr JmpRefT jmp_fwd_y_dec() { return jmp_fwd(JmpCond::Y_Dec); }
	constexpr JmpRefT jmp_fwd_x_ne_y() { return jmp_fwd(JmpCond::X_NE_Y); }
	constexpr JmpRefT jmp_fwd_pin() { return jmp_fwd(JmpCond::Pin); }
	constexpr JmpRefT jmp_fwd_not_osre() { return jmp_fwd(JmpCond::Not_OSRE); }

	constexpr void jmp_dest(JmpRefT jmp_instr) { jmp_instr.jump_to(index); }

	// ---- WAIT ----
	constexpr InstrRefT wait(const bool pol, const WaitSrc src, const uint8_t idx) {
		return get_instr(0x01, pol << 2 | to_underlying(src), idx);
	}

	constexpr InstrRefT wait_gpio(const bool pol, const uint8_t idx) {
		return wait(pol, WaitSrc::GPIO, idx);
	}
	constexpr InstrRefT wait_pin(const bool pol, const uint8_t idx) {
		return wait(pol, WaitSrc::Pin, idx);
	}
	constexpr InstrRefT wait_irq(const bool pol, const uint8_t idx) {
		return wait(pol, WaitSrc::IRQ, idx);
	}

	// ---- IN ----
	constexpr InstrRefT in(const InSrc src, const uint8_t bit_count) {
		// Bit count of 32 is encoded as 0x00. Masking is less safe but fast
		return get_instr(0x02, to_underlying(src), bit_count & 0x1F);
	}

	constexpr InstrRefT in_pins(const uint8_t bit_count) { return in(InSrc::Pins, bit_count); }
	constexpr InstrRefT int_x(const uint8_t bit_count) { return in(InSrc::X, bit_count); }
	constexpr InstrRefT in_y(const uint8_t bit_count) { return in(InSrc::Y, bit_count); }
	constexpr InstrRefT in_null(const uint8_t bit_count) { return in(InSrc::Null, bit_count); }
	constexpr InstrRefT in_isr(const uint8_t bit_count) { return in(InSrc::ISR, bit_count); }
	constexpr InstrRefT in_osr(const uint8_t bit_count) { return in(InSrc::OSR, bit_count); }

	// ---- OUT ----
	constexpr InstrRefT out(const OutDest dest, const uint8_t bit_count) {
		// Bit count of 32 is encoded as 0x00. Masking is less safe but fast
		return get_instr(0x03, to_underlying(dest), bit_count & 0x1F);
	}

	constexpr InstrRefT out_pins(const uint8_t bit_count) { return out(OutDest::Pins, bit_count); }
	constexpr InstrRefT out_x(const uint8_t bit_count) { return out(OutDest::X, bit_count); }
	constexpr InstrRefT out_y(const uint8_t bit_count) { return out(OutDest::Y, bit_count); }
	constexpr InstrRefT out_null(const uint8_t bit_count) { return out(OutDest::Null, bit_count); }
	constexpr InstrRefT out_pindirs(const uint8_t bit_count) {
		return out(OutDest::PinDirs, bit_count);
	}
	constexpr InstrRefT out_pc(const uint8_t bit_count) { return out(OutDest::PC, bit_count); }
	constexpr InstrRefT out_isr(const uint8_t bit_count) { return out(OutDest::ISR, bit_count); }
	constexpr InstrRefT out_exec(const uint8_t bit_count) { return out(OutDest::Exec, bit_count); }

	// ---- PUSH/PULL ----
	constexpr InstrRefT push(const bool if_full = false, const bool block = true) {
		return get_instr(0x04, if_full << 1 | block, 0x00);
	}

	constexpr InstrRefT pull(const bool if_empty = false, const bool block = true) {
		return get_instr(0x04, 0x04 | if_empty << 1 | block, 0x00);
	}

	constexpr InstrRefT push_iffull() { return push(true); }
	constexpr InstrRefT push_noblock() { return push(false, false); }
	constexpr InstrRefT push_block() { return push(false, true); }
	constexpr InstrRefT push_iffull_noblock() { return push(true, false); }
	constexpr InstrRefT push_iffull_block() { return push(true, true); }

	constexpr InstrRefT pull_iffull() { return pull(true); }
	constexpr InstrRefT pull_noblock() { return pull(false, false); }
	constexpr InstrRefT pull_block() { return pull(false, true); }
	constexpr InstrRefT pull_iffull_noblock() { return pull(true, false); }
	constexpr InstrRefT pull_iffull_block() { return pull(true, true); }

	// ---- MOV ----
	constexpr InstrRefT mov(const MovDest dest, const MovSrc src, const MovOp op = MovOp::None) {
		return get_instr(0x05, to_underlying(dest), to_underlying(op) << 3 | to_underlying(src));
	}

	constexpr InstrRefT mov_not(const MovDest dest, const MovSrc src) {
		return mov(dest, src, MovOp::Not);
	}
	constexpr InstrRefT mov_reverse(const MovDest dest, const MovSrc src) {
		return mov(dest, src, MovOp::BitReverse);
	}

	// ---- IRQ ----
	constexpr InstrRefT irq(
	    const uint8_t index, const bool clear = false, const bool wait = false,
	    const bool rel = false) {
		PIO_ASSERT(index < 8);
		return get_instr(0x06, clear << 1 | wait, rel << 4 | index);
	}

	constexpr InstrRefT irq_set(const uint8_t index) { return irq(index, false, false, false); }
	constexpr InstrRefT irq_nowait(const uint8_t index) { return irq(index, false, false, false); }
	constexpr InstrRefT irq_wait(const uint8_t index) { return irq(index, false, true, false); }
	constexpr InstrRefT irq_clear(const uint8_t index) { return irq(index, true, false, false); }

	constexpr InstrRefT irq_set_rel(const uint8_t index) { return irq(index, false, false, true); }
	constexpr InstrRefT irq_nowait_rel(const uint8_t index) {
		return irq(index, false, false, true);
	}
	constexpr InstrRefT irq_wait_rel(const uint8_t index) { return irq(index, false, true, true); }
	constexpr InstrRefT irq_clear_rel(const uint8_t index) { return irq(index, true, false, true); }

	// ---- SET ----
	constexpr InstrRefT set(const SetDest dest, const uint8_t data) {
		return get_instr(0x07, to_underlying(dest), data);
	}

	constexpr InstrRefT set_pins(const uint8_t data) { return set(SetDest::Pins, data); }
	constexpr InstrRefT set_x(const uint8_t data) { return set(SetDest::X, data); }
	constexpr InstrRefT set_y(const uint8_t data) { return set(SetDest::Y, data); }
	constexpr InstrRefT set_pindirs(const uint8_t data) { return set(SetDest::PinDirs, data); }

	// ---- NOP ----
	constexpr InstrRefT nop() { return mov(MovDest::Y, MovSrc::Y); }

	// ---- Wrap ----
	constexpr void wrap_target() {
		PIO_ASSERT(index < 32);
		wrap_target_idx = index;
	}

	constexpr void wrap() {
		PIO_ASSERT(index <= 32);
		wrap_idx = index - 1;
	}

private:
	static_assert(SIDE_SET_PINS <= 5, "PIO Programs cannot have more than 5 side set bits");
	static_assert(
	    !SIDE_SET_OPT || SIDE_SET_PINS <= 4,
	    "PIO Programs cannot have more than 4 side set bits when side_set opt is enabled");
	static_assert(
	    !SIDE_SET_OPT || SIDE_SET_PINS, "If side_set opt is enabled, there must be side_set pins");

	__attribute__((always_inline)) constexpr InstrRefT get_instr(
	    const uint8_t opc, const uint8_t field_3, const uint8_t field_5) {
		PIO_ASSERT(opc < 8);
		PIO_ASSERT(field_3 < 8);
		PIO_ASSERT(field_5 < 32);
		PIO_ASSERT(index < 32);
		instructions[index] = opc << 13 | field_3 << 5 | field_5;
		return instructions[index++];
	}

	// Convert enum value to underlying type - This should be elided and is just to
	// make the type checker happy
	template <typename T>
	static constexpr std::underlying_type_t<T> to_underlying(const T& item) {
		return static_cast<std::underlying_type_t<T>>(item);
	}

public:
	std::array<uint16_t, 32> instructions;
	pio_program_t program = {.instructions = instructions.data(), .origin = -1};

	uint8_t index           = 0;
	uint8_t wrap_target_idx = 0;
	uint8_t wrap_idx        = 31;
};

} // namespace PIOBuilder

#undef PIO_ASSERT

#endif // PIO_BUILDER_HPP
