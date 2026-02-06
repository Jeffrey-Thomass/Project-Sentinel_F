# Project-Sentinel_F


# Priority-Aware Sensor Deployment using Discrete Optimization

This project implements a priority-aware sensor deployment system that optimizes sensor placement to maximize coverage of important regions while minimizing redundant overlap and sensor usage. The system is designed to help visualize and understand how discrete optimization techniques can be applied to real-world sensor deployment problems.

The project combines a constraint-based optimization backend with an interactive web-based visualization interface.

---

## Problem Overview

In many applications such as environmental monitoring, surveillance, and IoT systems, sensors are limited in number due to cost, energy, and deployment constraints. Poor sensor placement can lead to uncovered regions or unnecessary overlap, reducing overall efficiency.

This project addresses the problem by:
- Prioritizing important regions within the deployment area
- Maximizing coverage of these regions
- Limiting the number of sensors used
- Reducing redundant sensing overlap

---

## Key Features

- Priority-aware sensor deployment optimization
- Discretization of continuous priority regions into sub-target points
- Grid-based candidate sensor placement
- Constraint-based optimization using Google OR-Tools (CP-SAT)
- Interactive web-based visualization
- Real-time experimentation with sensor parameters

---

## System Architecture

The system follows a client–server architecture:

### Frontend
- Web-based interactive UI
- Allows users to:
  - Define environment size
  - Specify priority regions
  - Adjust sensor range and sensor budget
  - Visualize optimized sensor deployments in real time

### Backend
- Implemented using Python and Flask
- Uses Google OR-Tools CP-SAT solver
- Handles:
  - Environment discretization
  - Coverage computation
  - Optimization modeling and solving
- Returns optimized sensor placement and coverage results via API

---

## Optimization Approach

1. **Environment Discretization**
   - Priority regions are discretized into sub-target points.
   - Each sub-target represents a location that must be covered.

2. **Candidate Sensor Locations**
   - Sensors are restricted to predefined grid locations.
   - Grid density can be adjusted to control solution granularity.

3. **Decision Variables**
   - Binary variables determine:
     - Whether a sensor is placed at a grid location
     - Whether a sub-target is covered

4. **Constraints**
   - Sensor budget constraint limits the number of deployed sensors
   - Coverage constraints ensure sub-targets are covered by sensors within range

5. **Objective Function**
   - Maximize coverage of priority regions
   - Penalize excessive sensor usage
   - Penalize redundant overlap

6. **Solver**
   - The CP-SAT solver from Google OR-Tools is used to compute optimal or near-optimal solutions.

---

## Technologies Used

- Python
- Flask
- Google OR-Tools (CP-SAT Solver)
- NumPy
- HTML, CSS, JavaScript (Frontend Visualization)
- Canvas-based rendering

---

## How to Run the Project

### Prerequisites
- Python 3.x
- pip

### Install Dependencies
```bash
pip install flask flask-cors numpy ortools