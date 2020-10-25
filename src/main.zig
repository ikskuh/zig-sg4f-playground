const std = @import("std");
const lpc = @import("lpc1786");

const F_CPU = 100_000_000;

comptime {
    // Provides the initialization routines
    _ = @import("boot.zig");
}

const IO_ENA_24V = GPIO(0, 0);
const IO_BEL_OUT = GPIO(1, 19);

const IO_DISP_OE = GPIO(1, 25);
const IO_DISP_RST = GPIO(1, 22);

pub fn main() !void {
    // Enable GPIOs for basic debugging
    IO_ENA_24V.setDirection(.out);
    IO_BEL_OUT.setDirection(.out);

    IO_BEL_OUT.clear();
    IO_ENA_24V.set();

    PinConfig.configure(0, 0, .Func00, .PullUp, false); // P0.0 → GPIO (Enable 24V)
    PinConfig.configure(0, 2, .Func01, .PullUp, false); // P0.2 → UART0.TXD
    PinConfig.configure(0, 3, .Func01, .PullUp, false); // P0.3 → UART0.RXD
    PinConfig.configure(1, 19, .Func00, .PullUp, false); // P1.19 → GPIO/BEL_OUT

    PinConfig.configure(1, 20, .Func11, .PullUp, false); // P1.20 → SCK0
    PinConfig.configure(1, 21, .Func11, .PullUp, false); // P1.21 → SSEL0

    PinConfig.configure(1, 22, .Func00, .PullUp, false); // P1.22 → GPIO/DISP_RST

    PinConfig.configure(1, 23, .Func11, .PullUp, false); // P1.23 → MISO0
    PinConfig.configure(1, 24, .Func11, .PullUp, false); // P1.24 → MOSI0
    PinConfig.configure(1, 25, .Func00, .PullUp, false); // P1.25 → GPIO/DISP_OE

    // Enable PLL & serial for debug output
    PLL.init();
    Serial.init(19_200);

    var sync_serial = Serial.syncWriter();
    try sync_serial.writeAll("Serial port initialized.\r\n");

    SysTick.init(1_000);
    try sync_serial.writeAll("SysTick initialized.\r\n");

    EventLoop.init();

    try sync_serial.writeAll("Starting event loop...\r\n");

    var serial_frame = async doSerialLoop();
    var blinky_frame = async doBlinkyLoop();
    var display_frame = async doDisplayLoop();

    EventLoop.run();

    try nosuspend await serial_frame;
    nosuspend await blinky_frame;
    try nosuspend await display_frame;

    try sync_serial.writeAll("Event loop finished!\r\n");
}

fn doSerialLoop() !void {
    var buffer: [256]u8 = undefined;
    var out = Serial.writer();
    var in = Serial.reader();
    while (true) {
        try out.writeAll("Enter your name: ");
        const name = (try in.readUntilDelimiterOrEof(&buffer, '\r')) orelse unreachable;
        try out.print("\r\nYour name is {}. Hello, {}!\r\n", .{ name, name });
    }
}

fn doBlinkyLoop() void {
    while (true) {
        IO_BEL_OUT.set();
        EventLoop.waitForMillis(500);
        IO_BEL_OUT.clear();
        EventLoop.waitForMillis(250);
    }
}

fn doDisplayLoop() !void {
    SPI.init();

    Display.init();
    Display.fill(Color.red);

    const clut = blk: {
        var table: [8][8]Color = undefined;
        for (table) |*row, y| {
            for (row) |*c, x| {
                c.* = Color{
                    .r = @truncate(u5, x << 2),
                    .g = @truncate(u6, y << 2),
                    .b = 0,
                };
            }
        }
        break :blk table;
    };

    Display.force_move(0, 0);
    Display.begin_put();

    var z: u5 = 0;
    while (true) : (z +%= 5) {
        var start = SysTick.get();

        var y: u16 = 0;
        while (y < Display.height) : (y += 1) {
            var x: u16 = 0;
            while (x < Display.width) : (x += 1) {
                Display.put(clut[(y + 2 * z) & 7][(x + z) & 7]);
            }
        }

        var end = SysTick.get();

        try Serial.syncWriter().print("total fill time: {}ms with {} SPI and a event loop preferring {}\r\n", .{
            end - start,
            SPI.mode,
            switch (EventLoop.fairness) {
                .prefer_others => "other tasks",
                .prefer_current => "the current task",
            },
        });
    }
}

pub fn panic(message: []const u8, maybe_stack_trace: ?*std.builtin.StackTrace) noreturn {
    lpc.__disable_irq();

    IO_BEL_OUT.set();
    var serial = Serial.syncWriter();
    serial.print("reached panic: {}\r\n", .{message}) catch unreachable;

    if (maybe_stack_trace) |stack_trace| {
        var frame_index: usize = 0;
        var frames_left: usize = std.math.min(stack_trace.index, stack_trace.instruction_addresses.len);

        while (frames_left != 0) : ({
            frames_left -= 1;
            frame_index = (frame_index + 1) % stack_trace.instruction_addresses.len;
        }) {
            const return_address = stack_trace.instruction_addresses[frame_index];
            serial.print("[{d}] 0x{X}\r\n", .{
                frames_left,
                return_address,
            }) catch unreachable;
        }
    }

    while (true) {
        lpc.__disable_irq();
        lpc.__disable_fault_irq();
        lpc.__WFE();
    }
}

const PinDirection = enum { in, out };

fn GPIO(comptime portIndex: u32, comptime index: u32) type {
    return struct {
        const port_number = port_index;
        const pin_number = index;
        const pin_mask: u32 = (1 << index);

        const io = @intToPtr(*volatile lpc.LPC_GPIO_TypeDef, lpc.LPC_GPIO_BASE + 0x00020 * portIndex);

        fn setDirection(d: PinDirection) void {
            switch (d) {
                .out => io.FIODIR |= pin_mask,
                .in => io.FIODIR &= ~pin_mask,
            }
        }

        fn direction() PinDirection {
            return if ((io.FIODIR & pin_mask) != 0) .out else .in;
        }

        fn getValue() bool {
            return (io.FIOPIN & pin_mask) != 0;
        }

        fn clear() void {
            io.FIOCLR = pin_mask;
        }

        fn set() void {
            io.FIOSET = pin_mask;
        }

        fn setTo(value: bool) void {
            if (value) {
                set();
            } else {
                clear();
            }
        }

        fn toggle() void {
            if (value()) {
                clear();
            } else {
                set();
            }
        }
    };
}

const PLL = struct {
    fn init() void {
        reset_overclocking();
    }

    fn reset_overclocking() void {
        overclock_flash(5); // 5 cycles access time
        overclock_pll(3); // 100 MHz
    }

    fn overclock_flash(timing: u8) void {
        lpc.LPC_SC.FLASHCFG = (@as(u32, timing - 1) << 12) | (lpc.LPC_SC.FLASHCFG & 0xFFFF0FFF);
    }

    inline fn feed_pll() void {
        lpc.LPC_SC.PLL0FEED = 0xAA; // mit anschliessendem FEED
        lpc.LPC_SC.PLL0FEED = 0x55;
    }

    fn overclock_pll(divider: u8) void {
        // PLL einrichten für RC
        lpc.LPC_SC.PLL0CON = 0; // PLL disconnect
        feed_pll();

        lpc.LPC_SC.CLKSRCSEL = 0x00; // RC-Oszillator als Quelle
        lpc.LPC_SC.PLL0CFG = ((1 << 16) | 74); // SysClk = (4MHz / 2) * (2 * 75) = 300 MHz
        lpc.LPC_SC.CCLKCFG = divider - 1; // CPU Takt = SysClk / divider

        feed_pll();

        lpc.LPC_SC.PLL0CON = 1; // PLL einschalten
        feed_pll();

        var i: usize = 0;
        while (i < 1_000) : (i += 1) {
            lpc.__NOP();
        }

        lpc.LPC_SC.PLL0CON = 3; // PLL connecten
        feed_pll();
    }
};

const PinConfig = struct {
    const Function = enum(u2) {
        Func00 = 0,
        Func01 = 1,
        Func10 = 2,
        Func11 = 3,
    };

    const Mode = enum(u2) {
        PullUp = 0,
        Repeater = 1,
        Floating = 2,
        PullDown = 3,
    };

    inline fn setup(val: *volatile u32, mask: u32, value: u32) void {
        val.* = (val.* & ~mask) | (value & mask);
    }

    inline fn selectFunction(port: u32, pin: u32, function: Function) void {
        const offset = (pin / 16);
        const index = @intCast(u5, (pin % 16) << 1);
        const mask = (@as(u32, 3) << index);
        const value = @as(u32, @enumToInt(function)) << index;
        switch (2 * port + offset) {
            0 => setup(&lpc.LPC_PINCON.PINSEL0, mask, value), // P0.0
            1 => setup(&lpc.LPC_PINCON.PINSEL1, mask, value), // P0.16
            2 => setup(&lpc.LPC_PINCON.PINSEL2, mask, value), // P1.0
            3 => setup(&lpc.LPC_PINCON.PINSEL3, mask, value), // P1.16
            4 => setup(&lpc.LPC_PINCON.PINSEL4, mask, value), // P2.0
            5 => setup(&lpc.LPC_PINCON.PINSEL5, mask, value), // P2.16
            6 => setup(&lpc.LPC_PINCON.PINSEL6, mask, value), // P3.0
            7 => setup(&lpc.LPC_PINCON.PINSEL7, mask, value), // P3.16
            8 => setup(&lpc.LPC_PINCON.PINSEL8, mask, value), // P4.0
            9 => setup(&lpc.LPC_PINCON.PINSEL9, mask, value), // P4.16
            else => {},
        }
    }

    inline fn setMode(port: u32, pin: u32, mode: Mode) void {
        const offset = (pin / 16);
        const index = @intCast(u5, pin % 16);
        const mask = (@as(u32, 3) << (2 * index));
        const value = (@as(u32, @enumToInt(mode)) << (2 * index));
        switch (2 * port + offset) {
            0 => setup(&lpc.LPC_PINCON.PINMODE0, mask, value), // P0.0
            1 => setup(&lpc.LPC_PINCON.PINMODE1, mask, value), // P0.16
            2 => setup(&lpc.LPC_PINCON.PINMODE2, mask, value), // P1.0
            3 => setup(&lpc.LPC_PINCON.PINMODE3, mask, value), // P1.16
            4 => setup(&lpc.LPC_PINCON.PINMODE4, mask, value), // P2.0
            5 => setup(&lpc.LPC_PINCON.PINMODE5, mask, value), // P2.16
            6 => setup(&lpc.LPC_PINCON.PINMODE6, mask, value), // P3.0
            7 => setup(&lpc.LPC_PINCON.PINMODE7, mask, value), // P3.16
            8 => setup(&lpc.LPC_PINCON.PINMODE8, mask, value), // P4.0
            9 => setup(&lpc.LPC_PINCON.PINMODE9, mask, value), // P4.16
            else => {},
        }
    }

    inline fn setOpenDrain(port: u32, pin: u32, enabled: bool) void {
        const index = @intCast(u5, pin % 16);
        const mask = (@as(u32, 1) << index);
        const value = if (enabled) mask else 0;
        switch (port) {
            0 => setup(&lpc.LPC_PINCON.PINMODE_OD0, mask, value), // P0.0
            1 => setup(&lpc.LPC_PINCON.PINMODE_OD1, mask, value), // P1.0
            2 => setup(&lpc.LPC_PINCON.PINMODE_OD2, mask, value), // P2.0
            3 => setup(&lpc.LPC_PINCON.PINMODE_OD3, mask, value), // P3.0
            4 => setup(&lpc.LPC_PINCON.PINMODE_OD4, mask, value), // P4.0
            else => {},
        }
    }

    inline fn configure(port: u32, pin: u32, func: Function, mode: Mode, open_drain: bool) void {
        selectFunction(port, pin, func);
        setMode(port, pin, mode);
        setOpenDrain(port, pin, open_drain);
    }
};

const Serial = struct {
    const Error = error{};

    const SyncWriter = std.io.Writer(void, Error, writeSync);
    fn syncWriter() SyncWriter {
        return SyncWriter{ .context = {} };
    }

    const SyncReader = std.io.Reader(void, Error, readSync);
    fn syncReader() SyncReader {
        return SyncReader{ .context = {} };
    }

    const AsyncWriter = std.io.Writer(void, Error, writeAsync);
    fn writer() AsyncWriter {
        return AsyncWriter{ .context = {} };
    }

    const AsyncReader = std.io.Reader(void, Error, readAsync);
    fn reader() AsyncReader {
        return AsyncReader{ .context = {} };
    }

    fn init(comptime baudrate: u32) void {
        lpc.LPC_SC.PCONP |= (1 << 3); // PCUART0
        lpc.LPC_SC.PCLKSEL0 &= ~@as(u32, 0xC0);
        lpc.LPC_SC.PCLKSEL0 |= @as(u32, 0x00); // UART0 PCLK = SysClock / 4

        lpc.LPC_UART0.LCR = 0x83; // enable DLAB, 8N1
        lpc.LPC_UART0.unnamed_2.FCR = 0x00; // disable any fifoing

        const pclk = F_CPU / 4;
        const regval = (pclk / (16 * baudrate));

        lpc.LPC_UART0.unnamed_0.DLL = @truncate(u8, regval >> 0x00);
        lpc.LPC_UART0.unnamed_1.DLM = @truncate(u8, regval >> 0x08);

        lpc.LPC_UART0.LCR &= ~@as(u8, 0x80); // disable DLAB
    }

    fn tx(ch: u8) void {
        while ((lpc.LPC_UART0.LSR & (1 << 5)) == 0) {} // Wait for Previous transmission
        lpc.LPC_UART0.unnamed_0.THR = ch; // Load the data to be transmitted
    }

    fn available() bool {
        return (lpc.LPC_UART0.LSR & (1 << 0)) != 0;
    }

    fn rx() u8 {
        while ((lpc.LPC_UART0.LSR & (1 << 0)) == 0) {} // Wait till the data is received
        return @truncate(u8, lpc.LPC_UART0.unnamed_0.RBR); // Read received data
    }

    fn writeSync(context: void, data: []const u8) Error!usize {
        for (data) |c| {
            tx(c);
        }
        return data.len;
    }

    fn readSync(context: void, data: []u8) Error!usize {
        for (data) |*c| {
            c.* = rx();
        }
        return data.len;
    }

    fn writeAsync(context: void, data: []const u8) Error!usize {
        for (data) |c| {
            EventLoop.waitForRegister(u8, &lpc.LPC_UART0.LSR, (1 << 5), (1 << 5));

            std.debug.assert((lpc.LPC_UART0.LSR & (1 << 5)) != 0);
            lpc.LPC_UART0.unnamed_0.THR = c; // Load the data to be transmitted
        }
        return data.len;
    }

    fn readAsync(context: void, data: []u8) Error!usize {
        for (data) |*c| {
            EventLoop.waitForRegister(u8, &lpc.LPC_UART0.LSR, (1 << 0), (1 << 0));
            std.debug.assert((lpc.LPC_UART0.LSR & (1 << 0)) != 0);
            c.* = @truncate(u8, lpc.LPC_UART0.unnamed_0.RBR); // Read received data
        }
        return data.len;
    }
};

const SysTick = struct {
    var counter: u32 = 0;

    fn init(comptime freq: u32) void {
        lpc.NVIC_SetHandler(.SysTick, SysTickHandler);
        lpc.SysTick_Config(F_CPU / freq) catch unreachable;
    }

    fn get() u32 {
        return @atomicLoad(u32, &SysTick.counter, .SeqCst);
    }

    fn SysTickHandler() callconv(.Interrupt) void {
        _ = @atomicRmw(u32, &counter, .Add, 1, .SeqCst);
    }
};

const EventLoop = struct {
    const Fairness = enum {
        prefer_current,
        prefer_others,
    };
    const fairness: Fairness = .prefer_current;

    const SuspendedTask = struct {
        frame: anyframe,
        condition: WaitCondition,
    };

    const WaitCondition = union(enum) {
        register8: Register(u8),
        register16: Register(u16),
        register32: Register(u32),
        time: u32,

        fn Register(comptime T: type) type {
            return struct {
                register: *volatile T,
                mask: T,
                value: T,
            };
        }

        fn isMet(cond: @This()) bool {
            return switch (cond) {
                .register8 => |reg| (reg.register.* & reg.mask) == reg.value,
                .register16 => |reg| (reg.register.* & reg.mask) == reg.value,
                .register32 => |reg| (reg.register.* & reg.mask) == reg.value,
                .time => |time| SysTick.get() >= time,
            };
        }
    };

    var tasks: [64]SuspendedTask = undefined;
    var task_count: usize = 0;

    fn waitFor(condition: WaitCondition) void {
        // don't suspend if we already meet the condition
        if (fairness == .prefer_current) {
            if (condition.isMet())
                return;
        }
        std.debug.assert(task_count < tasks.len);

        var offset = task_count;
        tasks[offset] = SuspendedTask{
            .frame = @frame(),
            .condition = condition,
        };
        task_count += 1;

        suspend;
    }

    fn waitForRegister(comptime Type: type, register: *volatile Type, mask: Type, value: Type) void {
        std.debug.assert((mask & value) == value);

        var reg = WaitCondition.Register(Type){
            .register = register,
            .mask = mask,
            .value = value,
        };

        waitFor(switch (Type) {
            u8 => WaitCondition{
                .register8 = reg,
            },
            u16 => WaitCondition{
                .register16 = reg,
            },
            u32 => WaitCondition{
                .register32 = reg,
            },
            else => @compileError("Type must be u8, u16 or u32!"),
        });
    }

    fn waitForMillis(delta: u32) void {
        waitFor(WaitCondition{
            .time = SysTick.get() + delta,
        });
    }

    fn init() void {
        task_count = 0;
    }

    fn run() void {
        while (task_count > 0) {
            std.debug.assert(task_count <= tasks.len);
            var i: usize = 0;
            while (i < task_count) : (i += 1) {
                if (tasks[i].condition.isMet()) {
                    var frame = tasks[i].frame;
                    if (i < (task_count - 1)) {
                        std.mem.swap(SuspendedTask, &tasks[i], &tasks[task_count - 1]);
                    }
                    task_count -= 1;
                    resume frame;
                    break;
                }
            }
        }
    }
};

const SPI = struct {
    const Mode = enum {
        @"async",
        sync,
        fast,
    };

    const mode: Mode = .sync;

    const TFE = (1 << 0);
    const TNF = (1 << 1);
    const RNE = (1 << 2);
    const RFF = (1 << 3);
    const BSY = (1 << 4);

    fn init() void {
        lpc.LPC_SC.PCONP |= (1 << 21); // power on SSP0

        // SSP0 prescaler = 1 (CCLK)
        lpc.LPC_SC.PCLKSEL1 &= ~@as(u32, 3 << 10);
        lpc.LPC_SC.PCLKSEL1 |= @as(u32, 1 << 10);

        lpc.LPC_SSP0.CR0 = 0x0008; // kein SPI-CLK Teiler, CPHA=0, CPOL=0, SPI, 9 bit
        lpc.LPC_SSP0.CR1 = 0x02; // SSP0 an Bus aktiv

        setPrescaler(2);
    }

    fn xmit(value: u16) u16 {
        switch (mode) {
            .@"async" => EventLoop.waitForRegister(u32, &lpc.LPC_SSP0.SR, BSY, 0),
            .sync => while ((lpc.LPC_SSP0.SR & BSY) != 0) {}, // while not transmit fifo empty
            .fast => {},
        }
        lpc.LPC_SSP0.DR = value & 0x1FF;

        switch (mode) {
            .@"async" => EventLoop.waitForRegister(u32, &lpc.LPC_SSP0.SR, BSY, 0),
            .sync => while ((lpc.LPC_SSP0.SR & BSY) != 0) {}, // while not transmit fifo empty
            .fast => {},
        }
        return lpc.LPC_SSP0.DR;
    }

fn write(value: u16) void {
    switch (mode) {
        .@"async" => EventLoop.waitForRegister(u32, &lpc.LPC_SSP0.SR, TNF, TNF),
        .@"sync" => while ((lpc.LPC_SSP0.SR & TNF) == 0) {}, // while transmit fifo is full
        .fast => {},
    }
    lpc.LPC_SSP0.DR = value & 0x1FF;
}

    fn setPrescaler(prescaler: u32) void {
        lpc.LPC_SSP0.CPSR = prescaler;
    }
};

const Color = packed struct {
    b: u5,
    g: u6,
    r: u5,

    pub const red = Color{ .r = 0x1F, .g = 0x00, .b = 0x00 };
    pub const green = Color{ .r = 0x00, .g = 0x3F, .b = 0x00 };
    pub const blue = Color{ .r = 0x00, .g = 0x00, .b = 0x1F };
    pub const yellow = Color{ .r = 0x1F, .g = 0x3F, .b = 0x00 };
    pub const magenta = Color{ .r = 0x1F, .g = 0x00, .b = 0x1F };
    pub const cyan = Color{ .r = 0x00, .g = 0x3F, .b = 0x1F };
    pub const black = Color{ .r = 0x00, .g = 0x00, .b = 0x00 };
    pub const white = Color{ .r = 0x1F, .g = 0x3F, .b = 0x1F };
};

comptime {
    std.debug.assert(@sizeOf(Color) == 2);
}

const Display = struct {
    const width = 320;
    const height = 240;

    const MajorIncrement = enum {
        column_major,
        row_major,
    };
    const IncrementDirection = enum {
        decrement,
        increment,
    };

    const GCTRL_RAMWR = 0x22; // Set or get GRAM data
    const GCTRL_RAMRD = 0x22; // Set or get GRAM data
    const GCTRL_DISP_CTRL = 0x07;
    const GCTRL_DEVICE_CODE = 0x00; // rd only

    const GCTRL_V_WIN_ADR = 0x44; // Vertical range address end,begin
    const GCTRL_H_WIN_ADR_STRT = 0x45; // begin
    const GCTRL_H_WIN_ADR_END = 0x46; // end
    const GCTRL_RAM_ADR_X = 0x4E; // Set GRAM address x
    const GCTRL_RAM_ADR_Y = 0x4F; // Set GRAM address y

    // GCTRL_DISP_CTRL */
    const GDISP_ON = 0x0033;
    const GDISP_OFF = 0x0030;

    const GDISP_WRITE_PIXEL = 0x22;

    const DisplayCommand = struct {
        index: u8,
        value: u16,

        fn init(index: u8, value: u16) DisplayCommand {
            return DisplayCommand{
                .index = index,
                .value = value,
            };
        }
    };

    // Array of configuration descriptors, the registers are initialized in the order given in the table
    const init_commands = [_]DisplayCommand{
        // DLC-Parameter START
        DisplayCommand.init(0x28, 0x0006), // set SS and SM bit
        DisplayCommand.init(0x00, 0x0001), // start oscillator
        DisplayCommand.init(0x10, 0x0000), // sleep mode = 0
        DisplayCommand.init(0x07, 0x0033), // Resize register
        DisplayCommand.init(0x02, 0x0600), // RGB interface setting
        DisplayCommand.init(0x03, 0xaaae), // Frame marker Position  0x686a
        DisplayCommand.init(0x01, 0x30F0), // RGB interface polarity 70ef, setup BGR

        // *************Power On sequence ****************
        DisplayCommand.init(0x0f, 0x0000), // SAP, BT[3:0], AP, DSTB, SLP, STB
        DisplayCommand.init(0x0b, 0x5208), // VREG1OUT voltage   0x5408
        DisplayCommand.init(0x0c, 0x0004), // VDV[4:0] for VCOM amplitude
        DisplayCommand.init(0x2a, 0x09d5),
        DisplayCommand.init(0x0d, 0x000e), // SAP, BT[3:0], AP, DSTB, SLP, STB
        DisplayCommand.init(0x0e, 0x2700), // DC1[2:0], DC0[2:0], VC[2:0]    0X3200
        DisplayCommand.init(0x1e, 0x00ad), // Internal reference voltage= Vci;	 ac

        //set window
        DisplayCommand.init(0x44, 0xef00), // Set VDV[4:0] for VCOM amplitude
        DisplayCommand.init(0x45, 0x0000), // Set VCM[5:0] for VCOMH
        DisplayCommand.init(0x46, 0x013f), // Set Frame Rate
        DisplayCommand.init(0x4e, 0x0000),
        DisplayCommand.init(0x4f, 0x0000),

        //--------------- Gamma control---------------//
        DisplayCommand.init(0x30, 0x0100), // GRAM horizontal Address
        DisplayCommand.init(0x31, 0x0000), // GRAM Vertical Address
        DisplayCommand.init(0x32, 0x0106),
        DisplayCommand.init(0x33, 0x0000),
        DisplayCommand.init(0x34, 0x0000),
        DisplayCommand.init(0x35, 0x0403),
        DisplayCommand.init(0x36, 0x0000),
        DisplayCommand.init(0x37, 0x0000),
        DisplayCommand.init(0x3a, 0x1100),
        DisplayCommand.init(0x3b, 0x0009),
        DisplayCommand.init(0x25, 0xf000), // DC1[2:0], DC0[2:0], VC[2:0]	e000
        DisplayCommand.init(0x26, 0x3800), //18	  30
        // DLC-Parameter ENDE

        // {GCTRL_DISP_CTRL, 0, GDISP_ON}  /* Reg 0x0007, turn disp on, to ease debug */
        DisplayCommand.init(GCTRL_DISP_CTRL, GDISP_OFF), // Reg 0x0007, turn dispoff during ram clear
    };

    var cursor_x: u16 = 0;
    var cursor_y: u16 = 0;
    var cursor_dx: IncrementDirection = undefined;
    var cursor_dy: IncrementDirection = undefined;
    var horiz_incr: bool = false;

    pub fn init() void {
        IO_DISP_RST.setDirection(.out);
        IO_DISP_OE.setDirection(.out);

        enable(false);
        EventLoop.waitForMillis(100);
        reset();
        EventLoop.waitForMillis(100);
        enable(true);

        for (init_commands) |cmd| {
            exec(cmd.index, cmd.value);
        }

        set_entry_mode(.row_major, .increment, .increment);

        on();

        write_cmd(GDISP_WRITE_PIXEL);
        force_move(0, 0);
    }

    pub fn reset() void {
        IO_DISP_RST.clear();
        EventLoop.waitForMillis(1);
        IO_DISP_RST.set();
    }

    pub inline fn on() void {
        exec(GCTRL_DISP_CTRL, GDISP_ON);
    }

    pub inline fn off() void {
        exec(GCTRL_DISP_CTRL, GDISP_OFF);
    }

    pub fn enable(enabled: bool) void {
        IO_DISP_OE.setTo(enabled);
    }

    const ColorCmd = struct {
        lower: u9,
        upper: u9,

        fn writeToDisplay(self: @This()) void {
            SPI.write(self.upper);
            SPI.write(self.lower);
        }
    };

    inline fn decodeColor(color: Color) ColorCmd {
        const bits = @bitCast(u16, color);
        return ColorCmd{
            .upper = 0x100 | @as(u9, @truncate(u8, bits >> 8)),
            .lower = 0x100 | @as(u9, @truncate(u8, bits)),
        };
    }

    pub fn fill(color: Color) void {
        const cmd = decodeColor(color);

        write_cmd(GDISP_WRITE_PIXEL);

        var i: usize = 0;
        while (i < width * height) : (i += 1) {
            cmd.writeToDisplay();
        }
    }

    pub fn set(x: u16, y: u16, color: Color) void {
        if (move(x, y))
            write_cmd(GDISP_WRITE_PIXEL);
        decodeColor(color).writeToDisplay();
    }

    /// Moves the draw cursor to (x,y).
    /// This function lazily moves the cursor and only updates
    /// Returns `true` when the display received
    pub fn move(x: u16, y: u16) bool {
        force_move(x, y);
        return true;
        // // rotate 180°
        // // x = Display::width  - x - 1;
        // // y = Display::height - y - 1;

        // if((x != cursor_x) or (y != cursor_y))
        // {
        // 	force_move(x, y);
        // 	if(horiz_incr)
        // 	{
        //         cursor_x = switch(cursor_dx) {
        //             .decrement => if(cursor_x == 0) width - 1 else cursor_x - 1,
        //             .increment => if(cursor_x == width - 1) 0 else cursor_x + 1,
        //             else => unreachable,
        //         }
        // 		if(cursor_x < 0 || cursor_x >= width)
        // 			cursor_y += cursor_dy;
        // 	}
        // 	else
        // 	{
        // 		cursor_y += cursor_dy;
        // 		if(cursor_x < 0 || cursor_x >= width)
        // 			cursor_x += cursor_dx;
        // 	}
        // 	cursor_x = (cursor_x + width) % width;
        // 	cursor_y = (cursor_y + height) % height;
        // 	return true;
        // }
        // else
        // {
        // 	return false;
        // }
    }

    pub fn set_entry_mode(major_increment: MajorIncrement, horizontal_dir: IncrementDirection, vertical_dir: IncrementDirection) void {

        // set entry mode, use vertical writing when COLUMN_MAJOR
        var cmd = DisplayCommand{
            .index = 0x11,

            .value = 0x6200,
        };

        if (major_increment == .column_major)
            cmd.value |= (1 << 3); // vertical first, horizontal second
        if (horizontal_dir == .increment)
            cmd.value |= (1 << 4); // inc x
        if (vertical_dir == .increment)
            cmd.value |= (1 << 5);

        exec(cmd.index, cmd.value);

        cursor_dx = horizontal_dir;
        cursor_dy = vertical_dir;
        horiz_incr = major_increment == .column_major;
    }

    /// Moves the display cursor. This will not use auto-increment buffering,
    /// but will always issue a command to the display.
    pub fn force_move(x: u16, y: u16) void {
        exec(GCTRL_RAM_ADR_X, x);
        exec(GCTRL_RAM_ADR_Y, y);
        cursor_x = x;
        cursor_y = y;
    }

    /// Low level display API:
    /// Writes a "write pixel data" command, but without pixel data.
    pub fn begin_put() void {
        write_cmd(GDISP_WRITE_PIXEL);
    }

    /// Low level display API:
    /// Writes raw pixel data to the display. Make sure the display
    /// is in "write pixel data" mode!
    pub fn put(color: Color) void {
        decodeColor(color).writeToDisplay();
    }

    /// Low level display API:
    /// Writes a display command.
    inline fn write_cmd(value: u8) void {
        SPI.write(value);
    }

    /// Low level display API:
    /// Writes a display data byte.
    inline fn write_data(value: u8) void {
        SPI.write(0x100 | @as(u16, value));
    }

    /// Low level display API:
    /// Executes a single command with a 16 bit parameter. Used to set registers.
    inline fn exec(cmd: u8, value: u16) void {
        write_cmd(cmd);
        write_data(@truncate(u8, value >> 8));
        write_data(@truncate(u8, value & 0xFF));
    }
};
