# ROS2 PID Controller — Mass-Spring-Damper Simulation

A ROS2 implementation of a PID controller applied to a mass-spring-damper system. Built as a ROS2 learning project, demonstrating nodes, topics, subscribers, services, and launch files using a physically meaningful control system.

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

## ROS2 Concepts Demonstrated

| Concept | Implementation |
|---------|---------------|
| Node | `PIDNode` class inheriting from `rclcpp::Node` |
| Publisher | Publishes current position to `/position` at 100Hz |
| Subscriber | Subscribes to `/setpoint` to receive target position at runtime |
| Service | `/reset` service resets simulation to initial conditions |
| Timer | Drives the simulation loop at 10ms intervals |
| Launch file | `pid.launch.py` starts the node with a single command |

## File Structure

```
pid_controller/
├── src/
│   ├── pid_node.cpp        # ROS2 node implementation
│   └── controller.cpp      # PID controller and spring system logic
├── include/pid_controller/
│   └── controller.h        # Struct definitions and function declarations
├── launch/
│   └── pid.launch.py       # Launch file
├── CMakeLists.txt          # Build configuration
└── package.xml             # ROS2 package metadata
```

## Building

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Running

```bash
ros2 launch pid_controller pid.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/position` | `std_msgs/Float64` | Current position $x(t)$ published at 100Hz |
| `/setpoint` | `std_msgs/Float64` | Target position $x_{\text{setpoint}}$, subscribe to update at runtime |

### Setting a Setpoint

```bash
ros2 topic pub /setpoint std_msgs/msg/Float64 "data: 5.0"
```

### Monitoring Position

```bash
ros2 topic echo /position
```

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/reset` | `std_srvs/Trigger` | Resets position, velocity, and PID state to initial conditions |

### Calling the Reset Service

```bash
ros2 service call /reset std_srvs/srv/Trigger
```

## Parameters

All parameters are set in `pid_node.cpp`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mass` | 1.0 | Mass of the object ($m$) |
| `k` | 1.0 | Spring constant ($k$) |
| `c` | 1.0 | Damping coefficient ($c$) |
| `Kp` | 5.0 | Proportional gain ($K_p$) |
| `Ki` | 45.0 | Integral gain ($K_i$) |
| `Kd` | 20.0 | Derivative gain ($K_d$) |
| `dt` | 0.01 | Time step ($\Delta t$) |
| `setpoint` | 0.0 | Initial target position |
