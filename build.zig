const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
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

    // Define cpu module
    const cpu_module = b.addModule("cpu", .{
        .root_source_file = b.path("src/cpu.zig"),
    });

    // CPU tests
    const cpu_tests = b.addTest(.{
        .root_source_file = b.path("test/cpu_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    cpu_tests.root_module.addImport("cpu", cpu_module);  // Add module to tests

    const run_cpu_tests = b.addRunArtifact(cpu_tests);
    test_step.dependOn(&run_cpu_tests.step);
}