// Inferred pseudocode from libZ1_x86_64.so disassembly.
// This is not original source code.

#include <stdbool.h>
#include <stddef.h>

typedef enum { CONTROL_KEYBOARD = 0, CONTROL_SDK = 1 } Control;

typedef struct CtrlComponents {
    // ... many fields ...
    Control ctrl;           // offset ~0x1d0 (inferred)
    bool isPlot;            // offset ~0x1e0 (inferred)
    int trajChoose;         // offset ~0x1e4 (inferred)
    size_t armType;         // offset ~0x1e8 (inferred)
    // ... more fields ...
} CtrlComponents;

// Constructor initializes defaults then calls configProcess.
void CtrlComponents_CtrlComponents(CtrlComponents* self, int argc, char** argv) {
    // Many fields zeroed; defaults set.
    self->ctrl = CONTROL_SDK;   // inferred: set to 1 in constructor
    self->trajChoose = 1;       // inferred
    self->armType = 0x24;       // inferred default arm type
    // ... other defaults ...

    CtrlComponents_configProcess(self, argc, argv);
}

// High-level structure of configProcess.
void CtrlComponents_configProcess(CtrlComponents* self, int argc, char** argv) {
    // Load ../config/config.xml first.
    // Parse config.xml for IP, Port, collision settings, gripper torque, etc.
    // (Keys inferred from .rodata: IP, Port, collision/open, collision/limitT,
    //  collision/load, grasp_max_torque, arm_type, traj_default, etc.)

    // If argc <= 1: treat as normal config-only run.
    // If argc == 3 and argv[2][0] == 'p': enable plotting (isPlot = true)
    // Else: print "[ERROR] Input parameters is error." and exit.

    // There is one place where ctrl is changed from SDK to KEYBOARD:
    // ctrl is set to KEYBOARD only in the argc == 3 path (after argument checks).
    // Otherwise ctrl remains SDK.

    // Version printing is triggered by "-v" or "--version" (strings exist).
    // In that path, it prints "Version z1_controller: 2.2.7" then attempts
    // to construct IOUDP; failure on permissions is expected in sandbox.

    // Also loads a secondary config file ("./config/test.xml") in some branch
    // after parameter parsing, then reads arm_type and default trajectory.

    // Key observed behavior:
    // - Default ctrl: SDK
    // - Keyboard ctrl only when argc == 3 and specific argv value
    // - Invalid args -> error + exit
}
