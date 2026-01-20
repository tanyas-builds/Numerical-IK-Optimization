# Numerical Optimization: Inverse Kinematics in Robotic Manipulators
A computational study comparing the efficiency of the Newton-Raphson numerical method against analytical solutions for 2-DOF planar manipulators. This repository features MATLAB implementations of Jacobian-based inverse kinematics , demonstrating the scalability of numerical optimization for high-degree-of-freedom robotic systems.
![GitHub License](https://img.shields.io/badge/license-MIT-blue)
![MATLAB](https://img.shields.io/badge/Made%20with-MATLAB-orange)
![Field](https://img.shields.io/badge/Field-Robotics%20%26%20Mechatronics-green)

## Project Overview
[cite_start]This repository investigates the efficiency of the **Newton-Raphson method** in solving inverse kinematics (IK) problems for robotic manipulators compared to traditional analytical solutions[cite: 5]. [cite_start]While analytical solutions are ideal for simple, non-redundant robots, numerical optimization offers a faster and more versatile approach for high-degree-of-freedom (DOF) systems where closed-form solutions are computationally expensive or unavailable[cite: 7, 8].

## Mathematical Foundation
[cite_start]The core of this project is the iterative refinement of joint coordinates to minimize the error between the current end-effector position and the desired pose[cite: 49, 50].

### Newton-Raphson Iteration
The algorithm is implemented using the following update rule:
$$\theta_{n+1} = \theta_{n} - J^{-1} \cdot Err$$
Where:
* [cite_start]**$J$**: The Jacobian matrix (partial derivatives of the forward kinematics)[cite: 80, 83].
* [cite_start]**$Err$**: The error vector between the desired and current position[cite: 80].


## Repository Structure
* [cite_start]`IK_NewtonMethod.m`: Core implementation of the Newton-Raphson algorithm with convergence plotting[cite: 60].
* `IK_NewtonRaphson_2dof.m`: Interactive script for solving 2-DOF planar IK with user-defined link lengths and target positions.
* [cite_start]`Analytical_Solution.m`: Closed-form solver used as a performance baseline[cite: 58].
* `IKRevised.m`: Optimized version of the numerical solver with refined error handling.

## Results
[cite_start]Computational experiments demonstrate that the numerical approach successfully converges to a tolerance of $1 \times 10^{-5}$ typically within a few iterations, proving its practical utility for real-world robotics applications[cite: 9].

## How to Use
1. Clone the repository: `git clone https://github.com/TanyaradzwaChinyai/Numerical-IK-Optimization.git`
2. Open `IK_NewtonRaphson_2dof.m` in MATLAB.
3. Run the script and enter your desired link lengths and target $(x, y)$ coordinates when prompted.

## Author
**Tanyaradzwa Chinyai**
Graduate Student | M.S. Electrical and Computer Engineering (2026)
