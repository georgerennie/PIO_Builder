// SPDX-FileCopyrightText: 2021 George Rennie
// SPDX-License-Identifier: BSD-3-Clause

#include <array>
#include <cinttypes>
#include "hardware/pio.h"

namespace PIOBuilder {

enum class JmpCond : uint8_t {
	Always        = 0x00,
	X_Zero        = 0x20,
	X_PostDec     = 0x40,
	Y_Zero        = 0x60,
	Y_PostDec     = 0x80,
	X_NotEqual_Y  = 0xA0,
	Pin           = 0xC0,
	OSRE_NotEmpty = 0xE0,
};

enum class WaitSrc : uint8_t { GPIO = 0x00, Pin = 0x20, IRQ = 0x40 };

enum class InSrc : uint8_t {
	Pins = 0x00,
	X    = 0x20,
	Y    = 0x40,
	Null = 0x60,
	ISR  = 0xC0,
	OSR  = 0xE0,
};

enum class OutDest : uint8_t {
	Pins    = 0x00,
	X       = 0x20,
	Y       = 0x40,
	Null    = 0x60,
	PinDirs = 0x80,
	PC      = 0xA0,
	ISR     = 0xC0,
	Exec    = 0xE0
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

enum class MovOp : uint8_t { None = 0x00, Invert = 0x08, BitReverse = 0x10, Reversed = 0x18 };

enum class MovDest : uint8_t {
	Pins = 0x00,
	X    = 0x20,
	Y    = 0x40,
	Exec = 0x80,
	PC   = 0xA0,
	ISR  = 0xC0,
	OSR  = 0xE0
};

enum class SetDest : uint8_t {
	Pins    = 0x00,
	X       = 0x01,
	Y       = 0x02,
	PinDirs = 0x80,
};

template <typename T, T DEFAULT>
struct NewType {
private:
	const T val{DEFAULT};

public:
	constexpr NewType() = default;
	constexpr NewType(const T val) : val{val} {}
	constexpr operator T() const { return val; }
};

struct Delay : NewType<uint8_t, 0> {
	using NewType::NewType;
};
struct SideSet : NewType<uint8_t, 0> {
	using NewType::NewType;
};
struct SideSetEn : NewType<bool, true> {
	using NewType::NewType;
};

template <
    uint8_t LEN = 32, uint8_t SIDE_SET_PINS = 0, bool SIDE_SET_OPT = false,
    bool SIDE_SET_PINDIRS = false>
class Program {
public:
	Program() {
		static_assert(LEN <= 32, "PIO Programs cannot be longer than 32 instructions");
		static_assert(SIDE_SET_PINS <= 5, "PIO Programs cannot have more than 5 side set bits");
	};

	constexpr operator const pio_program_t*() const { return &program; }

	constexpr uint16_t& operator[](std::size_t idx) { return instructions[idx]; }

	constexpr inline pio_sm_config get_default_config(uint16_t offset) const {
		auto config = pio_get_default_sm_config();
		sm_config_set_wrap(&config, offset + wrap_target_idx, offset + wrap_idx);
		sm_config_set_sideset(&config, SIDE_SET_PINS, SIDE_SET_OPT, SIDE_SET_PINDIRS);
		return config;
	}

	constexpr uint8_t jmp(
	    const JmpCond cond, const uint8_t addr = 0, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return jmp_at(current_idx++, cond, addr, delay, side_set, side_set_en);
	}

	constexpr uint8_t jmp_at(
	    const uint8_t instr_idx, const JmpCond cond, const uint8_t addr = 0,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		auto cond_u8 = static_cast<uint8_t>(cond);
		write_instr(instr_idx, 0x0000 | cond_u8 | addr, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr void set_jmp_dest(const uint8_t jmp_idx, const uint8_t new_addr) {
		instructions[jmp_idx] &= 0xFFE0;
		instructions[jmp_idx] |= new_addr;
	}

	constexpr uint8_t wait(
	    const bool polarity, const WaitSrc src, const uint8_t index, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return wait_at(current_idx++, polarity, src, index, delay, side_set, side_set_en);
	}

	constexpr uint8_t wait_at(
	    const uint8_t instr_idx, const bool polarity, const WaitSrc src, const uint8_t index,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		auto src_u8 = static_cast<uint8_t>(src);
		write_instr(
		    instr_idx, 0x2000 | polarity << 7 | src_u8 | index, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t in(
	    const InSrc src, const uint8_t bit_count, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return in_at(current_idx++, src, bit_count, delay, side_set, side_set_en);
	}

	constexpr uint8_t in_at(
	    const uint8_t instr_idx, const InSrc src, const uint8_t bit_count,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		auto src_u8 = static_cast<uint8_t>(src);
		write_instr(instr_idx, 0x4000 | src_u8 | bit_count, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t out(
	    const OutDest dest, const uint8_t bit_count, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return out_at(current_idx++, dest, bit_count, delay, side_set, side_set_en);
	}

	constexpr uint8_t out_at(
	    const uint8_t instr_idx, const OutDest dest, const uint8_t bit_count,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		auto dest_u8 = static_cast<uint8_t>(dest);
		write_instr(instr_idx, 0x6000 | dest_u8 | bit_count, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t push(
	    const bool block = 1, const bool if_full = 0, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return push_at(current_idx++, block, if_full, delay, side_set, side_set_en);
	}

	constexpr uint8_t push_at(
	    const uint8_t instr_idx, const bool if_full = 0, const bool block = 1,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		write_instr(instr_idx, 0x8000 | if_full << 6 | block << 5, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t pull(
	    const bool block = 1, const bool if_full = 0, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return pull_at(current_idx++, block, if_full, delay, side_set, side_set_en);
	}

	constexpr uint8_t pull_at(
	    const uint8_t instr_idx, const bool if_full = 0, const bool block = 1,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		write_instr(instr_idx, 0x8800 | if_full << 6 | block << 5, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t mov(
	    const MovDest dest, const MovSrc src, const MovOp op = MovOp::None,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		return mov_at(current_idx++, dest, src, op, delay, side_set, side_set_en);
	}

	constexpr uint8_t mov_at(
	    const uint8_t instr_idx, const MovDest dest, const MovSrc src, const MovOp op = MovOp::None,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		auto dest_u8 = static_cast<uint8_t>(dest);
		auto op_u8   = static_cast<uint8_t>(op);
		auto src_u8  = static_cast<uint8_t>(src);
		write_instr(instr_idx, 0xA000 | dest_u8 | op_u8 | src_u8, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t irq(
	    const uint8_t index, const bool clear = 0, const bool wait = 0, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return irq_at(current_idx++, index, clear, wait, delay, side_set, side_set_en);
	}

	constexpr uint8_t irq_at(
	    const uint8_t instr_idx, const uint8_t index, const bool clear = 0, const bool wait = 0,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		write_instr(
		    instr_idx, 0xC000 | index | clear << 6 | wait << 5, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr uint8_t nop(
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		return nop_at(current_idx++, delay, side_set, side_set_en);
	}

	constexpr uint8_t nop_at(
	    const uint8_t instr_idx, const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		return mov_at(instr_idx, MovDest::Y, MovSrc::Y, MovOp::None, delay, side_set, side_set_en);
	}

	constexpr uint8_t set(
	    const SetDest dest, const uint8_t data, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		return set_at(current_idx++, dest, data, delay, side_set, side_set_en);
	}

	constexpr uint8_t set_at(
	    const uint8_t instr_idx, const SetDest dest, const uint8_t data,
	    const Delay delay = Delay(), const SideSet side_set = SideSet(),
	    const SideSetEn side_set_en = SideSetEn()) {
		auto dest_u8 = static_cast<uint8_t>(dest);
		write_instr(instr_idx, 0xE000 | dest_u8 | data, delay, side_set, side_set_en);
		return instr_idx;
	}

	constexpr void set_origin(const uint8_t origin) { program.origin = origin; }
	constexpr void set_write_idx(const uint8_t idx) { current_idx = idx; }

	constexpr uint8_t wrap() { return wrap_at(current_idx - 1); }
	constexpr uint8_t wrap_at(const uint8_t new_wrap_idx) { return wrap_idx = new_wrap_idx; }

	constexpr uint8_t wrap_target() { return wrap_target_at(current_idx); }
	constexpr uint8_t wrap_target_at(const uint8_t new_wrap_target_idx) {
		return wrap_target_idx = new_wrap_target_idx;
	}

private:
	constexpr void write_instr(
	    const uint8_t instr_idx, const uint16_t instr, const Delay delay = Delay(),
	    const SideSet side_set = SideSet(), const SideSetEn side_set_en = SideSetEn()) {
		instructions[instr_idx] = instr | (delay << 8) | (side_set << (13 - SIDE_SET_PINS));
		if (SIDE_SET_OPT) {
			instructions[instr_idx] |= side_set_en << 12;
		}
	}

	std::array<uint16_t, LEN> instructions;
	pio_program_t program   = {.instructions = instructions.data(), .length = LEN, .origin = -1};
	uint8_t current_idx     = 0;
	uint8_t wrap_target_idx = 0;
	uint8_t wrap_idx        = LEN - 1;
};

} // namespace PIOBuilder
