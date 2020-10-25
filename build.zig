const std = @import("std");

pub fn build(b: *std.build.Builder) void {
    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();

    const exe = b.addExecutable("async-lpc", "src/main.zig");
    exe.addCSourceFile("src/init.c", &[_][]const u8{
        "-std=c11",
        "-Weverything",
        "-fno-sanitize=undefined",
    });
    exe.addPackage(std.build.Pkg{
        .name = "lpc1786",
        .path = "libs/lpc1786/lpc1786.zig",
    });
    exe.setTarget(std.zig.CrossTarget{
        .cpu_arch = .thumb,
        .cpu_model = .{
            .explicit = &std.Target.arm.cpu.cortex_m3,
        },
        .os_tag = .freestanding,
        .abi = .eabi,
    });
    exe.addIncludeDir("include");
    exe.setBuildMode(mode);
    exe.install();
    exe.setLinkerScriptPath("./src/linker.ld");

    const create_hex = b.addSystemCommand(&[_][]const u8{
        "arm-none-eabi-objcopy",
        "-R",
        "isr_ramvector",
        "-R",
        "stack",
        "-R",
        ".data",
        "-R",
        ".bss",
        "-R",
        ".debug_abbrev",
        "-O",
        "ihex",
        // "zig-cache/bin/async-lpc",
        // "async-lpc.hex",
    });
    create_hex.addArtifactArg(exe);
    create_hex.addArg("async-lpc.hex");

    const hex_step = b.step("hex", "Creates a flashable ihex file");
    hex_step.dependOn(&create_hex.step);

    // This is 100% machine dependant
    const run_flash = b.addSystemCommand(&[_][]const u8{
        "flash-magic",
        "COM(5, 115200)",
        "DEVICE(LPC1768, 0.000000, 0)",
        "HARDWARE(BOOTEXEC, 50, 100)",
        "ERASEUSED(Z:\\home\\felix\\projects\\lowlevel\\async-lpc\\async-lpc.hex, PROTECTISP)",
        "HEXFILE(Z:\\home\\felix\\projects\\lowlevel\\async-lpc\\async-lpc.hex, NOCHECKSUMS, NOFILL, PROTECTISP)",
    });
    run_flash.step.dependOn(&create_hex.step);

    const flash_step = b.step("flash", "Creates a hex file and flashes this.");
    flash_step.dependOn(&run_flash.step);

    const run_term = b.addSystemCommand(&[_][]const u8{
        "picocom",
        "--baud",
        "19200",
        "--lower-rts", // Disable programmer
        "--lower-dtr", // Disable reset
        "/dev/ttyUSB0",
    });

    const term_step = b.step("terminal", "Starts picocom on the correct port");
    term_step.dependOn(&run_term.step);
}
