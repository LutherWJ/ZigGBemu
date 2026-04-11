const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // 1. Define Internal Modules
    const ModuleDef = struct { name: []const u8, path: []const u8, mod: *std.Build.Module = undefined };
    var internal_modules = [_]ModuleDef{
        .{ .name = "hw", .path = "src/constants.zig" },
        .{ .name = "mbc", .path = "src/mbc.zig" },
        .{ .name = "io", .path = "src/io.zig" },
        .{ .name = "mmu", .path = "src/mmu.zig" },
        .{ .name = "cpu", .path = "src/cpu.zig" },
        .{ .name = "mbc0", .path = "src/mbc0.zig" },
        .{ .name = "mbc1", .path = "src/mbc1.zig" },
        .{ .name = "header", .path = "src/header.zig" },
        .{ .name = "joypad", .path = "src/joypad.zig" },
        .{ .name = "timer", .path = "src/timer.zig" },
        .{ .name = "interrupts", .path = "src/interrupts.zig" },
        .{ .name = "sdt", .path = "src/sdt.zig" },
        .{ .name = "ppu", .path = "src/ppu.zig" },
        .{ .name = "emulator", .path = "src/emulator.zig" },
        .{ .name = "pixel_fifo", .path = "src/pixel_fifo.zig" },
        .{ .name = "display", .path = "src/display.zig" },
    };

    // 2. Initialize and Link Modules
    for (&internal_modules) |*m| {
        m.mod = b.addModule(m.name, .{ .root_source_file = b.path(m.path) });
    }

    // Link every module to every other module (except itself)
    // This removes the need to manually specify dependencies between internal files.
    for (internal_modules) |m| {
        for (internal_modules) |other| {
            if (!std.mem.eql(u8, m.name, other.name)) {
                m.mod.addImport(other.name, other.mod);
            }
        }
    }

    // 3. Define Executable
    const exe = b.addExecutable(.{
        .name = "ZigGBemu",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Link all modules to the executable
    for (internal_modules) |m| {
        exe.root_module.addImport(m.name, m.mod);
    }

    b.installArtifact(exe);

    // Run command
    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    const run_step = b.step("run", "Run the emulator");
    run_step.dependOn(&run_cmd.step);

    // 4. Define Tests
    const test_step = b.step("test", "Run unit tests");

    const tests = [_]struct { name: []const u8, path: []const u8 }{
        .{ .name = "cpu", .path = "test/cpu_test.zig" },
        .{ .name = "mbc0", .path = "test/mbc0_test.zig" },
        .{ .name = "mbc1", .path = "test/mbc1_test.zig" },
        .{ .name = "timer", .path = "test/timer_test.zig" },
    };

    for (tests) |t| {
        const unit_test = b.addTest(.{
            .root_source_file = b.path(t.path),
            .target = target,
            .optimize = optimize,
        });

        // Link all internal modules to every test
        for (internal_modules) |m| {
            unit_test.root_module.addImport(m.name, m.mod);
        }

        const run_unit_test = b.addRunArtifact(unit_test);
        test_step.dependOn(&run_unit_test.step);
    }

    // 5. WASM Build
    const wasm_step = b.step("wasm", "Build the WASM binary for the web");
    const wasm = b.addExecutable(.{
        .name = "ziggbemu",
        .root_source_file = b.path("src/ffi.zig"),
        .target = b.resolveTargetQuery(.{
            .cpu_arch = .wasm32,
            .os_tag = .freestanding,
        }),
        .optimize = optimize,
    });

    // This ensures all exported functions in ffi.zig are visible to JavaScript
    wasm.rdynamic = true;
    wasm.entry = .disabled;

    for (internal_modules) |m| {
        wasm.root_module.addImport(m.name, m.mod);
    }

    // This copies the compiled wasm from the zig-cache directly into your web/ folder
    const copy_wasm = b.addUpdateSourceFiles();
    copy_wasm.addCopyFileToSource(wasm.getEmittedBin(), "web/ziggbemu.wasm");
    wasm_step.dependOn(&copy_wasm.step);
}
