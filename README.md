# Autonomous Box Search for Unitree G1 in MuJoCo

## Hackathon Task

### Objective
Develop an autonomous control system for the Unitree G1 humanoid robot in simulation that can:
1. Autonomously find a target object (box)
2. Use visual perception (mandatory)
3. Approach the object
4. Stop at a specified distance

### Environment Requirements
- Room: 4x4m square, flat floor, static walls, no obstacles, static lighting

### Target Object (Box)
- Shape: cube, 0.3x0.3x0.3m
- Placed on the floor
- Solid color (team's choice)
- Position unknown to algorithm, may change between runs
- **Hardcoding object coordinates is prohibited**

### Robot
- Model: Unitree G1, starts at room center, arbitrary initial orientation
- Allowed: model simplification, joint fixation, abstract base control

### Sensors
- RGB camera, LiDAR, IMU
- **Prohibited:** ground-truth position of the box, direct access to object state in simulator

### Control
- Allowed: standard Unitree locomotion API, movement commands (linear + angular)
- **Prohibited:** manual control, telepresence

### Success Criteria
- Fully autonomous behavior
- Visual object detection used
- Final distance to box <= 0.5m
- Solution robust to box position changes
- Custom-built scene

### Deliverables
1. Demo video (up to 2 minutes)
2. Source code
3. README with simulator description, scene description, launch instructions
4. Brief approach description (up to 1 page)

### Verification Procedure
- Box position will be changed
- Robot initial orientation will be changed
- Scene geometry must match task requirements

---

## Overview

Autonomous system for the Unitree G1 humanoid robot that finds and approaches a red cube in a 4x4m room using vision-based perception (RGB camera + depth buffer as LiDAR).

**Simulator:** MuJoCo 3.3.6
**Robot:** Unitree G1 (29 DOF)
**Language:** C++ (OpenCV + unitree_sdk2)

## Architecture

```
MuJoCo Simulator Process                    g1_ctrl Process
┌──────────────────────────────┐            ┌──────────────────────┐
│ PhysicsLoop (thread 1)       │            │                      │
│  ├─ mj_step (physics)        │            │ DDS sub:             │
│  └─ Offscreen head_cam render│            │  WirelessController  │
│     └─ OpenCV HSV detection  │   DDS      │  LowState            │
│     └─ Depth at cube bbox    │───────────>│                      │
│     └─ AutoSearch state      │            │ RL velocity policy   │
│        machine               │            │  joystick → velocity │
│                              │            │  → joint commands    │
│ BridgeThread (thread 2)      │            │                      │
│  ├─ AutoSearch → virtual     │<───────────│ DDS pub:             │
│  │   joystick override       │   DDS      │  LowCmd (motors)     │
│  ├─ LowCmd → mj ctrl        │            │                      │
│  └─ Sensors → LowState pub  │            └──────────────────────┘
└──────────────────────────────┘
```

The key insight: `g1_ctrl` does not know it's being controlled by an algorithm. AutoSearch generates virtual joystick values (`ly` for forward, `rx` for yaw), which are published via DDS as `WirelessController` — identical to a real gamepad.

## Scene

- **Room:** 4x4m square, flat floor, static walls, static lighting
- **Cube:** 0.3x0.3x0.3m red box (`rgba="0.8 0.2 0.2 1"`), free joint, random position at each launch
- **Robot:** Unitree G1 at center, random yaw orientation at each launch

## Sensors Used

| Sensor | Implementation | Purpose |
|--------|---------------|---------|
| RGB Camera | MuJoCo offscreen render (`head_cam`, 320x240, FOV 75°) | Red cube detection via OpenCV HSV filtering |
| LiDAR | MuJoCo `rangefinder` sensor on head + depth buffer at cube bbox center | Distance measurement for stopping at ≤0.5m |
| IMU | Built-in MuJoCo sensors | Balance and locomotion (used by g1_ctrl) |

**No ground-truth access** — cube position is determined solely through visual perception.

## Algorithm: AutoSearch State Machine

```
┌──────────────┐    cube not visible    ┌──────────────┐
│  SEARCHING   │◄──────────────────────│              │
│  rx = 1.0    │                        │              │
│  ly = 0.0    │    cube visible        │   OpenCV     │
│  (rotate     │───────────────────────>│   HSV        │
│   right)     │                        │   Detection  │
└──────┬───────┘                        └──────────────┘
       │ cube visible
       ▼
┌──────────────┐
│  CENTERING   │    |offset| > 0.15
│  rx = ±1.0   │    (cube not centered)
│  ly = 0.0    │
└──────┬───────┘
       │ |offset| ≤ 0.15 (cube centered)
       ▼
┌──────────────┐
│  APPROACHING │    depth > 0.5m
│  rx = 0.0    │
│  ly = 0.7    │
└──────┬───────┘
       │ depth ≤ 0.5m
       ▼
┌──────────────┐
│   STOPPED    │
│  rx = 0.0    │
│  ly = 0.0    │
└──────────────┘
```

### Detection Pipeline

1. Offscreen render `head_cam` → RGB 320x240 + depth buffer
2. RGB → BGR → HSV color space
3. Red color filter (H: 0-10 and 170-180, S: 80-255, V: 80-255)
4. Morphological open/close cleanup
5. `findContours` → largest contour → bounding box
6. Horizontal offset from image center → steering direction
7. Depth at bounding box center → distance to cube

## Files Modified/Created

| File | Change |
|------|--------|
| `unitree_robots/g1/g1_29dof.xml` | Added `head_cam` camera, LiDAR rangefinder on head; increased arm damping |
| `unitree_robots/g1/scene_room.xml` | Added red cube with free joint |
| `simulate/src/auto_search.h` | **NEW** — AutoSearch class (detection + state machine) |
| `simulate/src/main.cc` | Offscreen rendering, random init, auto_launch boot sequence, g1_ctrl subprocess |
| `simulate/src/unitree_sdk2_bridge.h` | DDS button injection for auto_launch, virtual joystick override |
| `simulate/src/param.h` | Config params: auto_launch, enable_auto_search, show_camera_window |
| `APPROACH.md` | **NEW** — Краткое описание подхода (1 страница) |
| `simulate/config.yaml` | New config entries |
| `simulate/CMakeLists.txt` | OpenCV dependency |

## Quick Start

```bash
# 1. Clone
git clone https://github.com/diaskabdualiev/physical_ai_championship.git
cd physical_ai_championship

# 2. Install system dependencies
sudo apt install libopencv-dev libglfw3-dev libboost-program-options-dev libyaml-cpp-dev libfmt-dev

# 3. Install unitree_sdk2
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
cd ../..

# 4. Build simulator (MuJoCo downloads automatically)
cd simulate && mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../..

# 5. Build controller
cd g1_ctrl && mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../..
```

Architecture (aarch64/x86_64) is auto-detected. MuJoCo 3.3.6 is downloaded automatically during `cmake`. ONNX Runtime for the controller is included in the repo for both architectures.

## Run

### Автономный режим (auto_launch=1, по умолчанию)

Одна команда — робот автоматически встаёт, включает Velocity mode и начинает поиск:

```bash
cd simulate/build
./unitree_mujoco
```

Контроллер `g1_ctrl` запускается автоматически как дочерний процесс. Джойстик не нужен.

### Ручной режим (auto_launch=0)

```bash
# Terminal 1
cd simulate/build
./unitree_mujoco

# Terminal 2
cd g1_ctrl/build
./g1_ctrl --network=lo
```

1. Gamepad: `LT + Up` → FixStand
2. Gamepad: `RB + X` → Velocity mode
3. Keyboard `0` → AutoSearch

**Debug window:** OpenCV window shows head camera view with detection overlay (SEARCHING / CENTERING / APPROACHING / STOPPED) and depth readout.

## Configuration

`simulate/config.yaml`:
```yaml
auto_launch: 1           # 1 = fully autonomous (single terminal, no gamepad)
use_joystick: 0          # 0 when auto_launch=1
enable_auto_search: 0    # auto_launch handles this automatically
show_camera_window: 1    # 1 to show OpenCV debug camera window
```

## Verification

At each launch:
- Cube spawns at random position (±1.5m from center, min 0.8m from robot)
- Robot yaw is randomized
- Robot finds cube through vision only (no ground-truth)
- Robot stops at ≤0.5m from cube (measured by depth sensor at cube bbox)
- Works regardless of cube position or robot orientation

## Approach Summary

Classical computer vision approach using HSV color filtering for red cube detection, combined with MuJoCo's depth buffer as a simulated LiDAR for distance estimation. The robot uses a simple state machine (search → center → approach → stop) with velocity commands sent through the standard Unitree joystick DDS interface. The architecture cleanly separates perception (AutoSearch) from locomotion (g1_ctrl), matching the real-robot deployment pattern where a vision node publishes commands consumed by the locomotion controller.
