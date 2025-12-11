# Flow Monitoring Optimization: ILP vs Greedy Set Cover

This project implements **switch placement optimization** for network flow monitoring.  
Given:
- A set of flows
- The path (set of switches) each flow traverses

The goal is:

> **Select the minimum number of switches such that every flow is observed at least once.**

This is a classic **Set Cover** optimization problem.

The repository provides:

- A **GLPK-based Integer Linear Programming (ILP)** solver for exact optimization  
- A **Greedy approximation solver** for large-scale datasets  
- A **10,000-flow, 100-switch sample dataset**  

---

## 📌 Problem Definition

We have:
- `n` flows: `f₁, f₂, …, fₙ`
- `m` switches: `s₁, s₂, …, sₘ`
- For each flow `i`, a list of switches it traverses: `Path(i)`

### Objective
Choose a minimum set of switches `S'` such that:


This is equivalent to the **Minimum Set Cover problem**, which is NP-hard.

---

## 🚀 ILP Formulation (Exact Optimization)

### Variables


### Constraints  
For each flow `i`:


### Objective  
minimize sum_j x_j


### Result
ILP finds the **optimal minimum number of switches**, but solving large datasets (e.g., 10k flows) can take hours due to NP-hard complexity.

---

## ⚡ Greedy Strategy (Approximation)

Greedy is fast and near-optimal:

1. Start with all flows uncovered
2. At each step, pick the switch covering the **largest number of uncovered flows**
3. Mark those flows as covered
4. Repeat until all flows are covered

Greedy runs in **milliseconds** on a 10k-flow dataset and generally stays within **5–10% of optimal**.

---

## 📂 Dataset Format

The dataset is stored as:
flow_id,switch_id
0,12
0,45
1,6
1,33
...


Each row means:

> *Flow `flow_id` traverses switch `switch_id`.*

The provided sample dataset contains:
- **10,000 flows**
- **100 switches**
- Random path lengths between 3 and 8

File: `flows_10k_100s.csv`

---

## 🛠️ Compilation (Windows + MSYS2)

### Compile ILP Solver
Requires GLPK:

```bash
g++ -std=gnu++17 set_cover_ilp.cpp -lglpk -O2 -o set_cover_ilp.exe

Compile Greedy Solver
g++ -std=gnu++17 greedy_set_cover.cpp -O2 -o greedy_set_cover.exe

▶️ Running the Solvers
Run ILP Solver
./set_cover_ilp.exe flows_10k_100s.csv


Output includes:

Optimal number of switches (if solvable)

Selected switch set

Flow coverage validation

Run Greedy Solver
./greedy_set_cover.exe flows_10k_100s.csv


Output includes:

Step-by-step selections

Total switches chosen

Runtime (in seconds)
