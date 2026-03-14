const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{
        .default_target = .{}
    });
    const optimize = b.standardOptimizeOption(.{});

    // Executable
    const exe = b.addExecutable(.{
        .name = "ZigGBemu",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(exe);

    // Run command
    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    const run_step = b.step("run", "Run the emulator");
    run_step.dependOn(&run_cmd.step);

    // Tests
    const test_step = b.step("test", "Run unit tests");

    const constants_module = b.addModule("hw", .{
        .root_source_file = b.path("src/constants.zig"),
    });

    const mbc_module = b.addModule("mbc", .{
        .root_source_file = b.path("src/mbc.zig"),
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

    // Set up module dependencies
    cpu_module.addImport("hw", constants_module);
    cpu_module.addImport("mmu.zig", mmu_module);
    cpu_module.addImport("mbc.zig", mbc_module);

    mmu_module.addImport("hw", constants_module);
    mmu_module.addImport("mbc.zig", mbc_module);

    const header_module = b.addModule("header.zig", .{
        .root_source_file = b.path("src/header.zig"),
    });
    header_module.addImport("hw", constants_module);

    mbc_module.addImport("mbc0.zig", mbc0_module);
    mbc_module.addImport("mbc1.zig", mbc1_module);
    mbc_module.addImport("header.zig", header_module);

    mbc0_module.addImport("hw", constants_module);
    mbc1_module.addImport("hw", constants_module);

    // CPU tests
    const cpu_tests = b.addTest(.{
        .root_source_file = b.path("test/cpu_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    cpu_tests.root_module.addImport("cpu", cpu_module);

    const run_cpu_tests = b.addRunArtifact(cpu_tests);
    test_step.dependOn(&run_cpu_tests.step);

    // MBC0 tests
    const mbc0_tests = b.addTest(.{
        .root_source_file = b.path("test/mbc0_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    mbc0_tests.root_module.addImport("mbc0", mbc0_module);
    mbc0_tests.root_module.addImport("constants", constants_module);

    const run_mbc0_tests = b.addRunArtifact(mbc0_tests);
    test_step.dependOn(&run_mbc0_tests.step);

    // MBC1 tests
    const mbc1_tests = b.addTest(.{
        .root_source_file = b.path("test/mbc1_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    mbc1_tests.root_module.addImport("mbc1", mbc1_module);
    mbc1_tests.root_module.addImport("constants", constants_module);

    const run_mbc1_tests = b.addRunArtifact(mbc1_tests);
    test_step.dependOn(&run_mbc1_tests.step);
}