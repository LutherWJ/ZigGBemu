const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // 1. Define Modules
    const constants_module = b.addModule("hw", .{
        .root_source_file = b.path("src/constants.zig"),
    });

    const mbc_module = b.addModule("mbc", .{
        .root_source_file = b.path("src/mbc.zig"),
    });

    const io_module = b.addModule("io", .{
        .root_source_file = b.path("src/io.zig"),
    });

    const mmu_module = b.addModule("mmu", .{
        .root_source_file = b.path("src/mmu.zig"),
    });

    const cpu_module = b.addModule("cpu", .{
        .root_source_file = b.path("src/cpu.zig"),
    });

    const mbc0_module = b.addModule("mbc0", .{
        .root_source_file = b.path("src/mbc0.zig"),
    });

    const mbc1_module = b.addModule("mbc1", .{
        .root_source_file = b.path("src/mbc1.zig"),
    });

    const header_module = b.addModule("header", .{
        .root_source_file = b.path("src/header.zig"),
    });

    const joypad_module = b.addModule("joypad", .{
        .root_source_file = b.path("src/joypad.zig"),
    });

    const timer_module = b.addModule("timer", .{
        .root_source_file = b.path("src/timer.zig"),
    });

    const interrupts_module = b.addModule("interrupts", .{
        .root_source_file = b.path("src/interrupts.zig"),
    });

    // 2. Define Module Dependencies (Important for internal imports)
    header_module.addImport("hw", constants_module);

    cpu_module.addImport("hw", constants_module);
    cpu_module.addImport("mmu", mmu_module);
    cpu_module.addImport("mbc", mbc_module);
    cpu_module.addImport("interrupts", interrupts_module);
    cpu_module.addImport("timer", timer_module);

    mmu_module.addImport("hw", constants_module);
    mmu_module.addImport("mbc", mbc_module);
    mmu_module.addImport("io", io_module);
    mmu_module.addImport("interrupts", interrupts_module);

    io_module.addImport("hw", constants_module);
    io_module.addImport("interrupts", interrupts_module);
    io_module.addImport("timer", timer_module);
    io_module.addImport("joypad", joypad_module);

    timer_module.addImport("hw", constants_module);
    timer_module.addImport("interrupts", interrupts_module);

    mbc_module.addImport("mbc0", mbc0_module);
    mbc_module.addImport("mbc1", mbc1_module);
    mbc_module.addImport("header", header_module);
    
    mbc0_module.addImport("hw", constants_module);
    mbc1_module.addImport("hw", constants_module);

    // 3. Define Executable
    const exe = b.addExecutable(.{
        .name = "ZigGBemu",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    
    // Link modules to the executable
    exe.root_module.addImport("hw", constants_module);
    exe.root_module.addImport("cpu", cpu_module);
    exe.root_module.addImport("mmu", mmu_module);
    exe.root_module.addImport("mbc", mbc_module);
    exe.root_module.addImport("io", io_module);
    
    b.installArtifact(exe);

    // Run command
    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    const run_step = b.step("run", "Run the emulator");
    run_step.dependOn(&run_cmd.step);

    // 4. Define Tests
    const test_step = b.step("test", "Run unit tests");

    const cpu_tests = b.addTest(.{
        .root_source_file = b.path("test/cpu_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    cpu_tests.root_module.addImport("cpu", cpu_module);
    cpu_tests.root_module.addImport("hw", constants_module);
    cpu_tests.root_module.addImport("mmu", mmu_module);
    cpu_tests.root_module.addImport("mbc", mbc_module);
    cpu_tests.root_module.addImport("io", io_module);
    cpu_tests.root_module.addImport("interrupts", interrupts_module);
    cpu_tests.root_module.addImport("timer", timer_module);
    cpu_tests.root_module.addImport("joypad", joypad_module);
    
    const run_cpu_tests = b.addRunArtifact(cpu_tests);
    test_step.dependOn(&run_cpu_tests.step);

    const mbc0_tests = b.addTest(.{
        .root_source_file = b.path("test/mbc0_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    mbc0_tests.root_module.addImport("mbc0", mbc0_module);
    mbc0_tests.root_module.addImport("hw", constants_module);
    
    const run_mbc0_tests = b.addRunArtifact(mbc0_tests);
    test_step.dependOn(&run_mbc0_tests.step);

    const mbc1_tests = b.addTest(.{
        .root_source_file = b.path("test/mbc1_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    mbc1_tests.root_module.addImport("mbc1", mbc1_module);
    mbc1_tests.root_module.addImport("hw", constants_module);
    
    const run_mbc1_tests = b.addRunArtifact(mbc1_tests);
    test_step.dependOn(&run_mbc1_tests.step);
}
