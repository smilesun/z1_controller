#include <csignal>
#include <sched.h>
#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Passive.h"
#include "FSM/State_BackToStart.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_Cartesian.h"
#include "FSM/State_JointSpace.h"
#include "FSM/State_MoveJ.h"
#include "FSM/State_MoveL.h"
#include "FSM/State_MoveC.h"
#include "FSM/State_ToState.h"
#include "FSM/State_SaveState.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_Trajectory.h"
#include "FSM/State_LowCmd.h"

bool running = true;
// controller/client that talks to a lower‑level motor controller over UDP (or ROS in sim), and the
// actual motor driving lives in the hardware/firmware and the prebuilt libZ1_*.so.
//
//  - include/message/udp.h defines a UDP socket transport (UDPPort) and raw send/recv buffers for 
//  7 motors.
//  - include/interface/IOUDP.h wraps UDP for send/recv of LowlevelCmd/LowlevelState.
//  - The sim path uses ROS topics in sim/IOROS.cpp

//set real-time program
void setProcessScheduler(){
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        // std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv){
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(5);


    EmptyAction emptyAction((int)ArmFSMStateName::INVALID);
    std::vector<KeyAction*> events;
    CtrlComponents *ctrlComp = new CtrlComponents(argc, argv);
    
    ctrlComp->dt = 1.0/250.;
    ctrlComp->armConfigPath =  "../config/";
    ctrlComp->stateCSV = new CSVTool("../config/savedArmStates.csv");
    ctrlComp->ioInter = new IOUDP(ctrlComp->ctrl_IP.c_str(), ctrlComp->ctrl_port);
    ctrlComp->geneObj();
    if(ctrlComp->ctrl == Control::SDK){
        ctrlComp->cmdPanel = new ARMSDK(events, emptyAction, "127.0.0.1", 8072, 8071, 0.002);
    }
    // 1. Default is Control::SDK in the struct definition. See include/control/CtrlComponents.h
    //    (the Control ctrl = Control::SDK; member). 
    //  2. main.cpp constructs CtrlComponents(argc, argv) on line 40, so any override happens inside that  constructor (likely via configProcess(argc, argv)). The actual constructor and configProcess implementation are not in the source tree; they’re inside the prebuilt shared library in lib/libZ1_x86_64.so. So from source alone, the only guaranteed behavior is: 
    //  - default is SDK
    //  - it can be overridden during construction based on argv or config, but the exact rule is hidden in the .so.
    // You can reverse‑inspect the .so with a mix of lightweight tooling and full decompilation:              
  // 1. Strings + symbol names (quickest)                                                                   
  //    - Tools: strings, nm -C, objdump -T, readelf -s
  //   - Goal: find readable error messages, config keys, CLI flags, or log strings.
  //    - This is how we can often spot --keyboard, --sdk, etc.
  // 2. Disassembly (medium effort)
  //    - Tools: objdump -d -C, radare2, gdb
  //    - Goal: inspect CtrlComponents::configProcess and see how ctrl is set.
  //    - You’d look for comparisons against string literals or argv parsing.
  // 3. Decompilation (most effective)
  //    - Tools: Ghidra (free) or IDA (paid)
  //    - Goal: reconstruct C‑like code of CtrlComponents::configProcess.
  //   - This usually reveals the exact logic for ctrl.
    else if(ctrlComp->ctrl == Control::KEYBOARD){
        events.push_back(new StateAction("`", (int)ArmFSMStateName::BACKTOSTART));
        events.push_back(new StateAction("1", (int)ArmFSMStateName::PASSIVE));
        events.push_back(new StateAction("2", (int)ArmFSMStateName::JOINTCTRL));
        events.push_back(new StateAction("3", (int)ArmFSMStateName::CARTESIAN));
        events.push_back(new StateAction("4", (int)ArmFSMStateName::MOVEJ));
        events.push_back(new StateAction("5", (int)ArmFSMStateName::MOVEL));
        events.push_back(new StateAction("6", (int)ArmFSMStateName::MOVEC));
        events.push_back(new StateAction("7", (int)ArmFSMStateName::TEACH));
        events.push_back(new StateAction("8", (int)ArmFSMStateName::TEACHREPEAT));
        events.push_back(new StateAction("9", (int)ArmFSMStateName::SAVESTATE));
        events.push_back(new StateAction("0", (int)ArmFSMStateName::TOSTATE));
        events.push_back(new StateAction("-", (int)ArmFSMStateName::TRAJECTORY));
        events.push_back(new StateAction("=", (int)ArmFSMStateName::CALIBRATION));

        events.push_back(new ValueAction("q", "a", 0.5));
        events.push_back(new ValueAction("w", "s", 0.5));
        events.push_back(new ValueAction("e", "d", 0.5));
        events.push_back(new ValueAction("r", "f", 0.5));
        events.push_back(new ValueAction("t", "g", 0.5));
        events.push_back(new ValueAction("y", "h", 0.5));
        events.push_back(new ValueAction("down", "up", 1.));

        ctrlComp->cmdPanel = new Keyboard(events, emptyAction);
    }
    std::vector<FSMState*> states;
    states.push_back(new State_Passive(ctrlComp));
    states.push_back(new State_BackToStart(ctrlComp));
    states.push_back(new State_JointSpace(ctrlComp));
    states.push_back(new State_Cartesian(ctrlComp));
    states.push_back(new State_MoveJ(ctrlComp));
    states.push_back(new State_MoveL(ctrlComp));
    states.push_back(new State_MoveC(ctrlComp));
    states.push_back(new State_LowCmd(ctrlComp));
    states.push_back(new State_SaveState(ctrlComp));
    states.push_back(new State_Teach(ctrlComp));
    states.push_back(new State_TeachRepeat(ctrlComp));
    states.push_back(new State_ToState(ctrlComp));
    states.push_back(new State_Trajectory(ctrlComp));
    states.push_back(new State_Calibration(ctrlComp));
    // - PASSIVE: motors relax / no active control.
  - JOINTCTRL (State_JointSpace): direct joint‑space control.
  - CARTESIAN: control end‑effector pose.
  - MOVEJ: point‑to‑point joint trajectory.
  - MOVEL: linear end‑effector trajectory.
  - MOVEC: circular end‑effector trajectory.
  - TRAJECTORY: follow a predefined trajectory.
  - BACKTOSTART: move to a known home pose.
  - CALIBRATION: calibration routine.
  - TEACH: record motions.
  - TEACHREPEAT: replay recorded motions.
  - SAVESTATE: save current pose to CSV.
  - TOSTATE: move to a saved pose.
  - LOWCMD: low‑level direct command state (usually SDK‑driven).


    FiniteStateMachine *fsm;
    fsm = new FiniteStateMachine(states, ctrlComp);
    // Where the FSM logic lives                                                                                                                 
    //- Declarations: include/FSM/FiniteStateMachine.h                                                                                    //  - Base state interface: include/FSM/FSMState.h, include/FSM/BaseState.h
    // - Implementation is in lib/libZ1_x86_64.so (not in source).  FiniteStateMachine::_run() and
    // FiniteStateMachine::FiniteStateMachine(...) are inside the .so.

    // Control flow (high‑level, reconstructed from the .so)

     // 1. Constructor (FiniteStateMachine::FiniteStateMachine)
     // - Copies the states vector into its own _states.
     // - Sets _currentState (first state in the list, which is State_Passive in main.cpp).
     // - Calls _currentState->enter() once.
     // - Creates a periodic Loop (from common/utilities/loop.h) bound to FiniteStateMachine::_run() and starts it.
     // 2. Loop callback (FiniteStateMachine::_run)
     // - Runs periodically (based on ctrlComp->dt).
     // - Calls the current state’s run() method.
     // - Gets the latest command (via ctrlComp->cmdPanel) and checks for transitions using currentState->checkChange(cmd).
     // - Applies safety overrides:
     // - FSMState::_collisionTest()
     // - LowlevelState::checkError()
     // - If a transition is requested:
     // - Calls currentState->exit()
     // - Switches _currentState to the next state
     // - Calls nextState->enter()


    ctrlComp->running = &running; // bool running = true;

    // SIGINT comes from <csignal> (Ctrl+C in terminal). `signal` (also in <csignal>)
    // registers a handler with signature void(int). The parameter `signum` is the
    // signal number delivered; it is unused here. Flipping `running` stops the
    // main loop and allows cleanup to run.
    signal(SIGINT, [](int signum){running = false;});
    // [](int signum) is the lambda function, here argument signum is not used

    std::this_thread::sleep_for(std::chrono::seconds(1));

    ctrlComp->cmdPanel->start();

    while(running){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //  Why that sleep is essential:
        //
        //    - It prevents a busy‑wait. Without the sleep, this loop would run as fast as possible
        //    and consume 100% CPU doing nothing.
        //    - The real control work runs in the FSM’s internal thread (created in the .so), so
        //      this loop is just a lifetime keeper. Sleeping yields
        //          CPU time while waiting for running to become false (e.g., SIGINT).
        //
        // Example in C++:
        //   std::thread t(worker);
        //  t.join(); // blocks until worker thread exits
        //
        // In this repo, the FSM loop runs in its own thread (created inside the .so). If we had direct 
        // access to that thread object, we could do:
        //  - fsm->join() or similar, and then the main thread would just block until the FSM stops
        //
    }

    delete fsm;
    delete ctrlComp;
    return 0;
}
