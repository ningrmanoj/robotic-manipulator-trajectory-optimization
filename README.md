# Robotic Manipulator Trajectory Optimization

## Overview

This project focuses on trajectory planning and dynamic analysis of a 2R (two-link) robotic manipulator using MATLAB.

The goal is to generate smooth joint trajectories, calculate the torques required to execute those motions, and optimize the motion time while ensuring that the actuator torque limits are not exceeded.

---

## Project Workflow

The project is divided into three main stages:

### 1. Trajectory Generation

**File:** `zvector_trajectorycomputer.m`

* Generates a smooth joint-space trajectory between a start and end position.
* Uses cubic spline interpolation.
* Computes:

  * Joint Position
  * Joint Velocity
  * Joint Acceleration
* Visualizes the generated motion profiles.

### 2. Inverse Dynamics Torque Calculation

**File:** `instant_torque_computer_galmani.m`

* Computes the torque required at each robot joint for a given motion state.

* Uses the dynamic model of the 2R manipulator:

  τ = M(q)q̈ + C(q,q̇)q̇ + G(q)

* Considers:

  * Inertia effects
  * Coriolis and centrifugal effects
  * Gravity effects

### 3. Time-Optimal Trajectory Optimization

**File:** `normalized_fminoptimization3.m`

* Optimizes the trajectory execution time.
* Enforces actuator torque limits during optimization.
* Uses Sequential Least Squares Programming (SLSQP).
* Produces:

  * Optimized trajectory
  * Joint torque profiles
  * Position, velocity, and acceleration plots

---

## Tools Used

* MATLAB
* Optimization Toolbox
* Cubic Spline Interpolation
* Sequential Least Squares Programming (SLSQP)

---

## Key Concepts

* Robot Dynamics
* Inverse Dynamics
* Trajectory Planning
* Motion Optimization
* Torque-Constrained Control
* Robotic Manipulators

---

## Team Information

* Team Size: 2 Members
* Project Type: Academic Robotics Project

---

## Future Improvements

* Obstacle avoidance
* Multi-link robotic manipulators
* Real-time trajectory tracking
* Hardware implementation on physical robotic systems
