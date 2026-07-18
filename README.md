# roboArm

Arduino firmware that drives a **2-link planar robotic arm** to a target point in its workspace. Rather than solving the inverse-kinematics equations in closed form, the firmware treats "reach the target" as an optimization problem and walks the two joint angles downhill toward the solution using **gradient descent**.

## How it works

Given a target `(x, y)`, each pass of the control loop:

1. Reads the current joint angles from the two servos.
2. Computes where the end effector currently is (forward kinematics via rotation matrices).
3. Computes how the distance-to-target changes with respect to each joint angle (the gradient).
4. Steps each joint a small amount in the direction that reduces that distance.

Because step 4 runs inside the Arduino `loop()`, every iteration performs one gradient-descent step and the arm converges over successive passes until the end effector is within a set threshold of the target.

```mermaid
flowchart TD
    A["Read joint angles θ1, θ2"] --> B["Forward kinematics:<br/>end-effector position"]
    B --> C{"Within threshold<br/>of target?"}
    C -- Yes --> D["Hold position"]
    C -- No --> E["Compute distance gradient<br/>for each joint"]
    E --> F["Step each servo<br/>against the gradient"]
    F --> A
```

## Hardware

- Arduino-compatible board
- 2 × hobby servos (pulse range 500–2500 µs), one per joint
- A two-segment arm linkage (equal link lengths in the model)

## Tech stack

- **C++ / Arduino**
- **Servo** library for servo PWM control
- Serial output at 9600 baud for live debugging of angles, gradient, and position

## Files

| File | Role |
|---|---|
| `test/test.ino` | Main controller: forward kinematics, distance gradient (`findSlopes`), workspace clamping (`getBoundedTarget`), and the descent loop |
| `zero/zero.ino` | Homing sketch that drives the servos to a known start pose |

## Notes

- The target point is set in code (`target[]`) and is automatically clamped to the arm's reachable workspace.
- Link lengths are modeled as equal; tune `armlen` and the step size (`0.04`) to adjust reach and convergence speed.
