# Reusable PID Controller in C

## Summary

This project features a reusable PID (Proportional-Integral-Derivative) controller implemented in C. It is designed using a structure that enables multiple instances to be reused effectively. This framework allows for testing code pieces in a C simulation environment, particularly suitable for control and embedded software engineering applications.

## Project Overview

The PID controller is a crucial component in control systems, widely used for its simplicity and effectiveness. This implementation focuses on creating a reusable and modular architecture. The project involves:

- **Object Model:** Simulates an object of mass $m = 10 \, \text{kg}$, subject to viscous friction with a coefficient $k = 0.5 \, \text{N}\cdot\text{s/m}$, and pushed by a force $F$.
- **PID Controller:** Implements a PID control loop that adjusts the control input to achieve desired object positioning.

### Architecture

The architecture of this program includes:

- **Model of an Object:**
  - Mass ($m$): 10 kg
  - Viscous friction coefficient ($k$): 0.5 N·s/m
  - Force ($F$): Limited to [-100, 100] N

- **PID Controller:**
  - Includes a filtered derivative for the derivative term
  - Command saturation and rate limiter
  - Anti-windup technique through back-calculation

### Mathematical Model

The model of the object is defined by the differential equation:

$$
m \cdot \frac{d^2z}{dt^2} = F - k \cdot \frac{dz}{dt}
$$

Where:
- $z$ is the position of the object in meters
- $v = \frac{dz}{dt}$ is the velocity of the object

### Simulation

The simulation environment is implemented in C and tests two slightly different PIDs controlling the position of two identical objects. The main function performs the following tasks:

- Creates structures for two PIDs and objects
- Iterates 1200 times (equivalent to 120 seconds of simulation time, with each step representing 0.1 seconds)
- Generates setpoints: 100 m and 50 m for the first 60 seconds, then changes to 200 m and 150 m
- Executes `PID_Step` using the previous step's $z$ value
- Executes `Object_Step` using the command from `PID_Step`

The simulation results are demonstrated graphically, showcasing the effectiveness of this approach.

## Features

- **Reusable Architecture:** Utilize structures to create multiple PIDs for different control tasks.
- **Efficient Simulation:** Run simulations with detailed output to analyze PID performance.
- **Comprehensive Control:** Incorporates derivative filtering, command saturation, rate limiting, and anti-windup.

## Usage

To use this PID controller framework:

1. Clone the repository: `git clone https://github.com/simorxb/PID-C-Struct`
2. Navigate to the directory: `cd PID-C-Struct`
3. Build the simulations with `mingw32-make` (MinGW on Windows) or `make`
4. Run the original simulation: `./Test_PID` (`Test_PID.exe` on Windows)

---

# Part 2 – General Back-Calculation Anti-Windup

## Summary

The PID and the mass-damper are now separate C modules, and the controller can run either classical back-calculation or general back-calculation. A comparison program runs the same tuning with both methods on the mass-damper, and a Python script plots position, saturated force, and unsaturated force.

## Project Overview

Classical back-calculation adds a term proportional to $u_{sat} - u$ onto the integrator only. With a filtered derivative the controller has another pole, so that correction does not cover the whole dynamic part.

General back-calculation splits the practical PID

$$
C(s) = K_p + \frac{K_i}{s} + K_d \frac{s}{T_C s + 1}
$$

into a direct term and a strictly proper dynamic term:

$$
C(\infty) = K_p + \frac{K_d}{T_C}, \qquad K = K_i T_C - \frac{K_d}{T_C}
$$

$$
\bar{C}(s) = \frac{K s + K_i}{s(T_C s + 1)}
$$

Where:
- $u$ is the unsaturated command
- $u_{sat}$ is the command after amplitude saturation and rate limiting
- $T_C$ is the derivative-filter time constant
- $K_{aw}$ is the anti-windup gain

The discrete realization follows the general back-calculation model: Tustin on $(K s + K_i)/(T_C s + 1)$, then a forward-Euler integrator. $K_{aw}(u_{sat} - u)$ is added to the error at the input of that dynamic part. `PID_BACK_CALCULATION_CLASSIC` keeps the original update, so `Test_PID` reproduces the original log.

### Controller Parameters

Both controllers in the comparison use the same plant and the same PID gains. The anti-windup gain is placed where each method uses it. Classical back-calculation adds $K_{aw}(u_{sat} - u)$ onto the integral state after $K_i$, so its gain is $K_i / C(\infty)$. General back-calculation uses $K_{aw} = 1/C(\infty)$.

- **Sample time**: $0.1~\text{s}$
- **Gains**: $K_p = 8$, $K_i = 0.5$, $K_d = 40$, $T_C = 0.5~\text{s}$
- **Direct term**: $C(\infty) = 88$
- **Force limits**: $\pm 25~\text{N}$, on both the controller and the object
- **Command rate limit**: $1000~\text{N/s}$, wide enough that it does not bind

## Simulation Setup

The reference ramps from $0$ to $20~\text{m}$ in $5~\text{s}$, holds until $40~\text{s}$, then ramps back to $0$ in $5~\text{s}$. The run lasts $80~\text{s}$. The plant is the same mass-damper as Part 1, $m = 10~\text{kg}$ and $k = 0.5~\text{N} \cdot \text{s/m}$.

On this saturated ramp, classical back-calculation overshoots by about $12\%$ and general back-calculation by about $6\%$.

## Files

- `pid.h`, `pid.c`: PID struct, classical and general `PID_Step`
- `object.h`, `object.c`: mass-damper struct and `Object_Step`
- `Test_PID.c`: original two-loop simulation, both controllers set to classical back-calculation
- `Test_PID_anti_windup_comparison.c`: one tuning, both anti-windup methods, logged to `data_PID_anti_windup_comparison.txt`
- `plot_anti_windup_comparison.py`: plots the comparison with NumPy and Matplotlib
- `Makefile`, `make.bat`: `make.bat` forwards to `mingw32-make` on Windows

Build and plot from the repository directory:

```text
mingw32-make
./Test_PID_anti_windup_comparison
python plot_anti_windup_comparison.py
```

## Key Takeaways

- **Classical back-calculation is unchanged.** `Test_PID` still logs the original two position loops.
- **General back-calculation corrects the dynamic part** of a PID with a filtered derivative, including the derivative filter.
- **Each method has its own $K_{aw}$.** Classical back-calculation uses $K_i/C(\infty)$ on the integral state. General back-calculation uses $1/C(\infty)$ at the input of the dynamic part.
- **The comparison is about amplitude saturation.** The rate limit is present and left wide open.

## Author

This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact

For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
