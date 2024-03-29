# PIO_Builder
Single header C++14 library for runtime generation of RPi RP2040 PIO programs

See `examples` for example code. `examples/i2c.cpp` provides a good example of
a more involved program, and the other examples show how to use PIOBuilder with
the pico sdk.

It can be included as a header in any other project using the pico sdk, and just
needs C++14 or later and to be linked against hardware_pio.

## Features

See the RP2040 documentation for details of the PIO design.

A program can optionally have its number of side set pins, side set opt and
side set pindirs set through template args, with defaults used otherwise
```cpp
PIOBuilder::Program<SIDE_SET_PINS = 0, SIDE_SET_OPT = false, SIDE_SET_PINDIRS = false>
```

To create your own program, create a class deriving from PIOBuilder:

```cpp
struct HelloPIO : public PIOBuilder::Program<> {
	HelloPIO() {
		auto loop = label();
		pull();
		out_pins(1);
		jmp(loop);
	}
};
```

The assembly language is meant to be very similar to pioasm.
See `pio_builder.hpp` to see what options are available for each command.

```cpp
pull_block(true);           // pull block
out_y(32);                  // out y, 32
mov(MovDest::X, MovSrc::Y); // mov x, y
```

Backward jumps can be created with a `label()` and `jmp_*` pair:

```cpp
auto loop = label();
nop();
jmp(loop);
```

Forward jumps can be created with the `jmp_fwd_*` variants of `jmp_*` functions
and a corresponding `jmp_dest`:

```cpp
auto jmp_target = jmp_fwd();
nop();
jmp_dest(jmp_target);
```

Delay, side set, wrap and wrap_target can be set in the same way as pioasm:

```cpp
wrap_target();
nop()    .side(1) [5];
nop()    .side(1) [5];
wrap();
```

The program can be loaded the PIOs similarly to the programs generated by
pioasm:

```cpp
HelloPIO my_program;

auto offset = pio_add_program(pio0, my_program);
// ...
pio_sm_config c = my_program.get_default_config(offset);
// ...
```
