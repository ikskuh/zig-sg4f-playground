const SuspendedTask = struct {
    prev: ?*SuspendedTask,
    next: ?*SuspendedTask,
    frame: anyframe,
    condition: WaitCondition,
};

var pending: ?*SuspendedTask = null;

fn waitFor(condition: WaitCondition) void {
    if (condition.isMet())
        return;

    var task = SuspendedTask{
        .prev = null,
        .next = pending,
        .frame = @frame(),
        .condition = condition,
    };

    if (pending) |p| {
        p.prev = &task;
    }

    pending = &task;
    suspend;
}

var ready: ?*SuspendedTask = null;

fn run() void {
    while (true) {
        var pending_task = pending orelse break;
        while (pending_task) |p| {
            const task = p;
            pending_task = task.next;
            if (task.condition.isMet()) {
                if (task.prev) |prev|
                    prev.next = task.next;
                if (task == pending)
                    pending = task.next;
                task.next = ready;
                ready = task;
            }
        }

        while (ready) |r| {
            const task = r;
            ready = task.next;
            resume task.frame;
        }
    }
}
