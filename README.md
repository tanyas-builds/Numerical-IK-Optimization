# Numerical IK Optimization: Newton-Raphson vs. Analytical Solvers
A computational study comparing the efficiency of the Newton-Raphson numerical method against analytical solutions for 2-DOF planar manipulators. This repository features MATLAB implementations of Jacobian-based inverse kinematics , demonstrating the scalability of numerical optimization for high-degree-of-freedom robotic systems.

![License](https://img.shields.io/badge/license-MIT-blue)
![MATLAB](https://img.shields.io/badge/Made%20with-MATLAB-orange)
![Field](https://img.shields.io/badge/Field-Robotics%20%26%20Mechatronics-green)

## Project Overview
[cite_start]This repository investigates the efficiency of the **Newton-Raphson numerical method** in solving inverse kinematics (IK) problems for robotic manipulators compared to traditional analytical solutions[cite: 4, 5]. [cite_start]While analytical solutions are ideal for simple, non-redundant robots, numerical optimization offers a versatile approach for high-degree-of-freedom (DOF) systems where closed-form solutions are computationally expensive or unavailable[cite: 7, 8].

## Mathematical Foundation
[cite_start]The core of this project is the iterative refinement of joint coordinates to minimize the error between the forward kinematics and the desired pose[cite: 49, 50].

### The Update Rule
[cite_start]The algorithm is implemented using the following update rule[cite: 79, 82]:
$$\theta_{n+1} = \theta_{n} - J^{-1} \cdot Err$$

### The Jacobian Matrix
[cite_start]For a 2-DOF planar manipulator, the Jacobian represents the partial derivatives of the forward kinematics with respect to the joint angles $\theta_1$ and $\theta_2$[cite: 83, 84].



$$J = \begin{bmatrix} -l_1\sin\theta_1-l_2\sin(\theta_1+\theta_2) & -l_2\sin(\theta_1+\theta_2) \\ l_1\cos\theta_1+l_2\cos(\theta_1+\theta_2) & l_2\cos(\theta_1+\theta_2) \end{bmatrix}$$

---

## Technical Specifications
[cite_start]Based on the computational experiments conducted in this study[cite: 58, 60]:

| Parameter | Value | Source File |
| :--- | :--- | :--- |
| **Link 1 Length ($l_1$)** | 1.0 - 200.0 units | `IK_NewtonMethod.m`, `IKRevised.m` |
| **Link 2 Length ($l_2$)** | 1.0 - 150.0 units | `IK_NewtonMethod.m`, `IKRevised.m` |
| **Convergence Tolerance** | $1 \times 10^{-5}$ | `IK_NewtonMethod.m` |
| **Iteration Limit** | 50 - 100 iterations | `IK_NewtonMethod.m`, `IKRevised.m` |
| **Initial Guesses** | $\pi/4, \pi/3$ | `IK_NewtonMethod.m` |

---

## Repository Structure
```text
├── assets/        # Visual aids and robot geometry diagrams
├── docs/          # Research paper (Numerical Optimization - Inverse Kinematics)
├── src/           # MATLAB Source Code
│   ├── Analytical_Solution.m   # Closed-form baseline solver
│   ├── IK_NewtonMethod.m       # Newton-Raphson implementation with plots
│   ├── IK_NewtonRaphson_2dof.m # Interactive solver with user inputs
│   └── IKRevised.m             # Refined numerical solver
├── LICENSE        # MIT License
└── README.md