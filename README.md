# PID Controller — Mass-Spring-Damper Simulation
A C++ implementation of a PID controller applied to a mass-spring-damper system. Written as a C++ learning project, translated from a conceptual understanding of control theory.

## Physics
The system models a mass on a spring with damping, governed by Newton's second law:

$$m\ddot{x} = F_{\text{applied}} - kx - c\dot{x}$$

Where:
- $m$ — mass
- $x$ — position
- $k$ — spring constant
- $c$ — damping coefficient
- $F_{\text{applied}}$ — force output from the PID controller

The equation is integrated forward in time using Euler integration:

$$\dot{x}_{n+1} = \dot{x}_n + \ddot{x}_n \cdot \Delta t$$

$$x_{n+1} = x_n + \dot{x}_n \cdot \Delta t$$

## PID Controller
The controller drives the system toward a target position (setpoint) by computing a corrective force from three terms:

$$F = K_p \cdot e(t) + K_i \int e(t)\, dt + K_d \frac{de(t)}{dt}$$

Where:
- $e(t) = x_{\text{setpoint}} - x(t)$ — current deviation from target
- $\int e(t)\, dt$ — accumulated error over time, corrects persistent steady-state offset
- $\dfrac{de(t)}{dt}$ — rate of change of error, dampens oscillation and overshoot
- $K_p$, $K_i$, $K_d$ — tuning constants

## File Structure
```
PID/
├── main.cpp          # Simulation loop, parameter setup, CSV output
├── controller.h      # Struct definitions and function declarations
├── controller.cpp    # PID controller and CSV helper implementation
├── CMakeLists.txt    # Build configuration
└── build/            # Compiled output (generated)
```

## Building
```bash
mkdir build
cd build
cmake ..
make
./PID
```
This produces `position.csv` and `time.csv` in the build directory.

## Parameters
All parameters are set in `main.cpp`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mass` | 1.0 | Mass of the object ($m$) |
| `k` | 1.0 | Spring constant ($k$) |
| `c` | 1.0 | Damping coefficient ($c$) |
| `Kp` | 1.0 | Proportional gain ($K_p$) |
| `Ki` | 1.0 | Integral gain ($K_i$) |
| `Kd` | 1.0 | Derivative gain ($K_d$) |
| `setpoint` | 5.0 | Target position ($x_{\text{setpoint}}$) |
| `dt` | 0.01 | Time step ($\Delta t$) |
| `steps` | 1000 | Number of simulation steps |


## Example Output
With default parameters the system starts at position $x = 1.0$, is driven toward the setpoint $x_{\text{setpoint}} = 5.0$, slightly overshoots around $t = 5$, then converges — a classic underdamped PID response.

Increasing $K_p$ produces more oscillation. Reducing $K_d$ increases overshoot. Adjusting $K_i$ corrects steady-state error.
