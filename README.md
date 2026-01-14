# Magnetic Levitation Control System

A Python implementation of a magnetic levitation system with PID controller.

## Files

- `dynamics.py` - Physics model for the electromagnetic ball levitation system
- `controller.py` - PID controller for stabilizing the ball

## Features

- Realistic physics simulation using RK4 integration
- Magnetic force model (inverse square law)
- RL circuit dynamics for electromagnet
- PID controller with tunable gains
- Voltage saturation and safety limits

## System Parameters

- Ball mass: 50g
- Coil resistance: 3Ω
- Coil inductance: 0.01H
- Magnetic force constant: 1e-4
- Voltage limits: ±50V
- Safe floor height: 1mm

## Usage

```python
from dynamics import Dynamics
from controller import PIDcontroller

# Initialize system
system = Dynamics()
controller = PIDcontroller(
    setpoint=0.05,  # 5cm
    kp=450,
    kd=100,
    ki=750,
    u_min=-50.0,
    u_max=50.0
)

# Define control function
def control_fn(t, state):
    y, ydot, i = state
    return controller.compute_control(y, ydot, system.dt_default)

# Run simulation
time, y, ydot, i, u = system.simulate(u_fn=control_fn, T=3.0)
```

## Future Work

- AI-based PID optimization (genetic algorithms, RL)
- Disturbance rejection testing
- Setpoint tracking scenarios
- Parameter uncertainty analysis
