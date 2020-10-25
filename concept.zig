const std = @import("std");

const AwaitTask = struct {
    frame: anyframe,
    register: *volatile u32,
    mask: u32,
    value: u32,

    fn isReady(task: @This()) bool {
        return (task.register.* & task.mask) == task.value;
    }
};

var tasks_backing_buffer: [64]*AwaitTask = undefined;
var tasks: []*AwaitTask = tasks_backing_buffer[0..0];

const AwaitState = union(enum) {
    running,
    suspended: AwaitTask,
    finished,
    dead,
};

fn waitForRegister(register: *volatile u32, mask: u32, value: u32) void {
    std.debug.assert((mask & value) == value);
    std.debug.assert(tasks.len < tasks_backing_buffer.len);

    var task = AwaitTask{
        .frame = @frame(),
        .register = register,
        .mask = mask,
        .value = value,
    };

    var offset = tasks.len;
    tasks.len += 1;
    tasks[offset] = &task;

    suspend;
}

pub fn main() !void {
    var serial_frame = async doSerialCommunication();
    var display_frame = async doDisplayCommunication();

    var iteration: usize = 0;
    while (tasks.len > 0) : (iteration += 1) {
        std.debug.print("                        idle loop iteration {}, awaiting {} tasks\n", .{ iteration, tasks.len });
        global_state += 1;

        var i: usize = 0;
        while (i < tasks.len) : (i += 1) {
            if (tasks[i].isReady()) {
                var frame = tasks[i].frame;
                if (i < (tasks.len - 1)) {
                    std.mem.swap(*AwaitTask, &tasks[i], &tasks[tasks.len - 1]);
                }
                tasks.len -= 1;
                resume frame;
                break;
            }
        }
    }

    nosuspend await serial_frame;
    nosuspend await display_frame;

    std.debug.print("we are done!\n", .{});
}

var global_state: u32 = 0;

fn serialWrite(string: []const u8) void {
    for (string) |c| {
        // wait that the serial port FIFO is ready to write
        std.debug.print("                                                                   serial pre-write: '{c}'\n", .{c});
        waitForRegister(&global_state, 0x07, 0x00);
        // std.debug.print("                                                                   serial post-write: '{c}'\n", .{c});
    }
}

fn fbusWrite(string: []const u8) void {
    for (string) |c| {
        // wait that the serial port FIFO is ready to write
        std.debug.print("                                                                                           fbus pre-write: '{c}'\n", .{c});
        waitForRegister(&global_state, 0x07, 0x04);
        // std.debug.print("                                                                                           fbus post-write: '{c}'\n", .{c});
    }
}

fn doSerialCommunication() void {
    serialWrite("NOW:");

    var com_a = async fbusWrite("DATA 1");
    var com_b = async serialWrite("COMPORT");

    await com_a;
    await com_b;
}

fn doDisplayCommunication() void {
    var i: usize = 0;
    while (i < 10) : (i += 1) {
        std.debug.print("display pre-suspend {}\n", .{i});
        waitForRegister(&global_state, 0x03, 0x02);
        // std.debug.print("display post-suspend {}\n", .{i});
    }
}
