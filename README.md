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
3. Compile the code using a C compiler: `gcc -o pid_simulation main.c`
4. Run the simulation: `./pid_simulation`

## Author

**Simone Bertoni**

For more information and projects, visit [Simone Bertoni Lab](https://simonebertonilab.com/).

## Contact

For further inquiries and discussions, connect on [LinkedIn](https://www.linkedin.com/in/simone-bertoni).

---

This project is a practical implementation of PID control theory, emphasizing flexibility and reusability in embedded systems development. It serves as a foundation for experimenting with and understanding control algorithms in a structured environment.
