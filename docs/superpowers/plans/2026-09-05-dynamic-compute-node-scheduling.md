# Dynamic Compute Node Scheduling Implementation Plan

> **For Codex:** Execute this plan in the current task using test-driven development and verify every completion claim with fresh command output.

**Goal:** Keep berth detection and SLAM as baseline nodes while automatically suspending bridge computation after 10 consecutive stable realtime berth timestamps and resuming it after 10 consecutive missing timestamps.

**Architecture:** Add a small deterministic scheduling policy independent of Qt threads, then let `MainWindow` translate policy decisions into one effective bridge-compute gate. Keep the bridge button as the manual master permission and add an internal processing gate to `BridgeDetectionNode` so dynamic standby drains work without destroying reusable worker threads.

**Tech Stack:** C++17, Qt 5, existing topic bus, CMake/CTest.

---

### Task 1: Specify the scheduling state machine with failing tests

**Files:**
- Create: `src/bus/DetectionNodeScheduler.h`
- Create: `tests/detection_node_scheduler_tests.cpp`
- Modify: `CMakeLists.txt`

1. Add tests for 10 stable timestamps, 10 missing timestamps, alternating observations, blind-recall exclusion, duplicate timestamps, timeline rollback, reset, and manual master permission.
2. Register the new test target and run it to confirm the expected failure before implementation.
3. Implement the smallest pure C++ policy and stable-realtime-result predicate.
4. Run the focused test until it passes.

### Task 2: Add configurable thresholds

**Files:**
- Modify: `src/core/AppConfig.h`
- Modify: `src/core/AppConfig.cpp`
- Modify: `config/config.json`
- Modify: `tests/detection_control_tests.cpp`

1. Add failing configuration assertions for the scheduler section.
2. Add defaults and JSON parsing for stable/missing timestamp thresholds.
3. Clamp invalid field values to a safe range.
4. Run the focused configuration/control tests.

### Task 3: Make bridge computation safely pausable

**Files:**
- Modify: `src/bus/BridgeDetectionNode.h`
- Modify: `src/bus/BridgeDetectionNode.cpp`
- Modify: `tests/detection_control_tests.cpp`

1. Add structural tests requiring a processing gate and pending-work invalidation.
2. Add `setProcessingEnabled`/state query APIs.
3. Gate topic enqueue, worker consumption, camera submissions, and late results; clear queued work on standby.
4. Keep full `stop()` for application shutdown only.

### Task 4: Integrate automatic scheduling and UI state

**Files:**
- Modify: `src/gui/mainwindow.h`
- Modify: `src/gui/mainwindow.cpp`
- Modify: `tests/detection_control_tests.cpp`

1. Split manual bridge permission from effective bridge-compute state.
2. Feed every berth result timestamp into the scheduler before drawing the berth overlay.
3. Apply transitions to traditional vision inputs, 210/211 inputs, near-point PCL processing, bridge-node processing, stale overlays, button text, status bar, and logs.
4. Reset scheduling on manual changes, PCAP time jumps, and new/offline/live stream starts.
5. Start berth detection and SLAM as the framework baseline while retaining the manual toolbar controls.

### Task 5: Verify regressions and deliver

**Files:**
- Verify only.

1. Configure and build the test target.
2. Run focused scheduling and detection-control tests.
3. Run the complete CTest suite.
4. Build the Release application and smoke-start it.
5. Report the exact behavior, configuration keys, and executable path.

