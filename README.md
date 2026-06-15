# Robotic Manipulator Trajectory Optimization

## Project Overview

This project investigates trajectory planning and time-optimal motion generation for a 2R robotic manipulator.

The workflow consists of:

1. Inverse dynamics torque computation
2. Cubic spline trajectory generation
3. Time-optimal trajectory optimization under actuator torque limits

The objective is to generate smooth robot motion while minimizing execution time and satisfying dynamic constraints.

## Technical Concepts

- Forward and Inverse Kinematics
- Manipulator Dynamics
- Cubic Spline Interpolation
- Trajectory Optimization
- Torque-Constrained Motion Planning
- Sequential Least Squares Programming (SLSQP)

## Tools

- MATLAB
- Optimization Toolbox
- Numerical Methods

## Team Size

2 Members

## Files

### inverse_dynamics_torque_calculator.m

Computes required joint torques using the manipulator dynamic model:

τ = M(q)q̈ + C(q,q̇)q̇ + G(q)

### cubic_spline_trajectory_generator.m

Generates smooth joint trajectories and computes:

- Position
- Velocity
- Acceleration

for both manipulator joints.

### time_optimal_trajectory_optimization.m

Minimizes trajectory execution time while enforcing torque constraints on both joints using nonlinear optimization.
