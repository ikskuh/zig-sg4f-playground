const std = @import("std");
const lpc1786 = @import("lpc1786");

const VECTOR_TABLE_LENGTH = 51;
const ISRHandler = fn () callconv(.Interrupt) void;

const VectorTable = [VECTOR_TABLE_LENGTH]ISRHandler;

extern var fixed_vector_table: VectorTable linksection(".isr_vector_ram");
var mutable_vector_table: VectorTable linksection(".isr_vector_ram") = undefined;

extern var __bss__start: c_void;
extern var __bss__end: c_void;
extern var __text__end: c_void;
extern var __data__start: c_void;
extern var __data__end: c_void;

export fn _start() callconv(.Interrupt) noreturn {
    std.mem.copy(ISRHandler, &mutable_vector_table, &fixed_vector_table);

    lpc1786.SCB.VTOR = @ptrToInt(&mutable_vector_table);
    lpc1786.SCB.SHP[7] = 1; // SVC has less priority than all fault handlers
    lpc1786.SCB.SHCSR = 0x00070000; // enable fault handler

    const bss = @ptrCast([*]u8, &__bss__start)[0 .. @ptrToInt(&__bss__end) - @ptrToInt(&__bss__end)];

    const ro_data = @ptrCast([*]const u8, &__text__end)[0 .. @ptrToInt(&__data__end) - @ptrToInt(&__data__start)];
    const rw_data = @ptrCast([*]u8, &__data__start)[0..ro_data.len];

    // BSS Segment löschen
    std.mem.set(u8, bss, 0);

    // Datasegment aus Flash in RAM kopieren
    std.mem.copy(u8, rw_data, ro_data);

    @import("root").main() catch |err| {
        @panic(@errorName(err));
    };
    while (true) {}
}

export fn _nmi() callconv(.Interrupt) void {
    @panic("nmi");
}

export fn _hardFault() callconv(.Interrupt) void {
    @panic("hard fault");
}

export fn _mpuFault() callconv(.Interrupt) void {
    @panic("mpu fault");
}

export fn _busFault() callconv(.Interrupt) void {
    @panic("bus fault");
}

export fn _usageFault() callconv(.Interrupt) void {
    @panic("usage fault");
}

export fn _unhandledInterrupt() callconv(.Interrupt) void {
    @panic("Unhandled interrupt!");
}
