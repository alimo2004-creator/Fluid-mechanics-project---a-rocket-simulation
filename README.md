# 🚀 Delta IV Launch Vehicle — Trajectory & Fluid Dynamics Simulation

> **Multidisciplinary simulation and analysis of a Delta IV Medium suborbital launch vehicle,**
> integrating computational fluid dynamics (CFD), structural CAD modelling, numerical trajectory optimization,
> and scale-prototype testing through dynamic similarity principles.

---

## 📋 Table of Contents

1. [Project Overview](#-project-overview)
2. [Repository Structure](#-repository-structure)
3. [MATLAB — Numerical Simulation & Trajectory Analysis](#-matlab--numerical-simulation--trajectory-analysis)
4. [SolidWorks — 3D CAD Modelling & Flow Add-In](#-solidworks--3d-cad-modelling--flow-add-in)
5. [ANSYS Fluent — Computational Fluid Dynamics (CFD)](#-ansys-fluent--computational-fluid-dynamics-cfd)
6. [Simulink — Prototype Simulation via Dynamic Similarity](#-simulink--prototype-simulation-via-dynamic-similarity)
7. [Key Results & Validation](#-key-results--validation)
8. [Dependencies & Software Versions](#-dependencies--software-versions)
9. [Authors & Acknowledgements](#-authors--acknowledgements)
10. [Link for the Drive](https://drive.google.com/drive/folders/1NqNussZyOPxgNKuYvnB-lACNyOmDsQzf?usp=drive_link)

---

## 🔭 Project Overview

This project presents a complete multi-tool engineering analysis of a **Delta IV Medium launch vehicle** designed to deliver a 10-ton communication satellite to an altitude of 200 km Low Earth Orbit (LEO) within exactly 300 seconds. The work spans the full simulation pipeline — from trajectory optimization and aerodynamic modeling in MATLAB, through high-fidelity 3D CAD and external flow analysis in SolidWorks, to full combustion exhaust CFD in ANSYS Fluent, and finally closed-loop prototype validation in Simulink using **dynamic similarity (dimensional analysis)**.

**Core objectives:**

- Optimize the exact partial-fueling requirements to reach a 200 km apogee using the RS-68A (Stage 1) and RL10B-2 (Stage 2) engines.
- Replace constant drag assumptions with a Mach-dependent variable drag coefficient $C_d(M)$ using slender-body aerodynamics.
- Validate theoretical flow parameters (exhaust velocity, Mach number, temperature) of the RS-68A nozzle against CFD results.
- Design a geometrically and dynamically similar scale prototype, with Simulink used to simulate and verify its transient response under matched non-dimensional conditions.

**Key propulsion parameters modelled:**

| Parameter | Value |
|---|---|
| Working Fluid | Liquid Hydrogen ($LH_2$) / Liquid Oxygen (LOX) |
| Stage 1 Engine (RS-68A) | $P_0$ = 9.7 MPa, $T_0$ = 3600 K, Thrust = 3116 kN |
| Stage 2 Engine (RL10B-2) | $P_0$ = 4.4 MPa, $T_0$ = 3300 K, Thrust = 110 kN |
| Target Orbit | 200 km Polar LEO |
| Prototype Scale Factor ($\lambda$) | 0.016 (Full scale: 62.5 m → Model: 1.0 m) |

![Delta IV 3D Model](images/3dmodel)

---

## 📁 Repository Structure

```text
Delta_IV_Fluid_Project/
│
├── MATLAB/
│   ├── Delta_IV_trial_2_variable_drag.m  # Main optimization and trajectory solver
│   ├── Delta_IV_VariableCd.csv           # Exported 0.1s time-step telemetry data
│   └── plots/                            # 9-panel telemetry dashboard images
│
├── SolidWorks/
│   ├── Rocket_Assembly.SLDASM            # Full 61m Delta IV CAD assembly
│   ├── FlowSim_Results/                  # SolidWorks Flow Simulation outputs
│   │   ├── velocity_trajectories.fld
│   │   ├── pressure_distribution.fld
│   │   └── Mach_contour.fld
│   └── Drawings/                         # Geometric dimensions and profiles
│
├── ANSYS_Fluent/
│   ├── mesh/
│   │   └── RS68A_nozzle_mesh.msh         # 2D axisymmetric structured nozzle mesh
│   ├── case_files/
│   │   └── RS68A_sea_level.cas           # Pressure inlet/outlet boundary case
│   └── results/
│       ├── velocity_mach_contour.png     # Exhaust velocity and oblique shocks
│       ├── temperature_contour.png       # 3600K to 240K gradient map
│       └── pressure_distribution.png     # Static pressure expansion
│
├── Simulink/
│   ├── prototype_dynamic_sim.slx         # Main Simulink model (Similarity Prototype)
│   ├── full_scale_reference.slx          # Full-scale flight reference scopes
│   └── results/
│       ├── altitude_vs_time.fig
│       ├── velocity_vs_time.fig
│       └── mass_vs_time.fig
│
├── images/                               # All visual assets for this README
├── Report/
│   └── FINAL_REPORT_TEAM_13.pdf
│
└── README.md
```

---

## 🧮 MATLAB — Numerical Simulation & Trajectory Analysis

### Overview

MATLAB serves as the **backbone of the optimization and trajectory simulation**. It handles continuous mass depletion, variable aerodynamics, and calculates the exact propellant mass required for the 300-second mission.

### 1. Mach-Dependent Drag Modelling

To replace a constant $C_d$, an array of **19 control points** spanning Mach 0 to 10 was created based on Delta IV CBC slender-body wind tunnel heritage.

- **Implementation:** A cubic spline interpolation (`pchip`) ensures a monotone shape during the transonic peak region, preventing oscillatory overshoots.
- **Result:** Accurately models the wave drag spike at Mach 1.0 and the transonic peak at Mach 1.1, asymptoting to Newtonian flow in the hypersonic regime.

![Mach-Dependent Drag Coefficient Curve](images/machcont)

### 2. Trajectory Optimization Loop

The code utilizes a **nested bisection optimization algorithm** to find the absolute minimum propellant mass.

- **Physics Engine:** Integrates Newton's second law with a continuous mass flow 4th-Order Runge-Kutta (RK4) solver.
- **Atmospheric Model:** Implements an exponential density decay model with a scale height of 8500 m and a variable gravity function $g(h)$.
- **Convergence:** The solver iterates through fuel loads with a 0.1-second time step. It successfully converged on **33,210.695 kg** of propellant per stage, achieving the 200 km apogee with a margin of error of just $\pm 0.3284$ meters.

![MATLAB Bisection Iterations Convergence](images/matlabiterations)

### 3. Telemetry Output & Dashboards

The script outputs a comprehensive CSV file logging iterations at each 0.1 s step and a **9-panel telemetry dashboard** tracking:

- Altitude & Velocity Profile
- G-Force Acceleration
- Vehicle Mass Depletion (MECO 1 & MECO 2 markers)
- Dynamic Pressure ($q$) and Max-Q detection

![Full Telemetry Dashboard — 9 Panel Output](images/finaloutput)

![Altitude vs Time Profile](images/altvsdown)

![Two-Stage Mass Depletion](images/twostages)

![Initial Trajectory Data](images/initialdata)

![CSV Final Export](images/csvfinal)

![CSV Data Table](images/csvdata)

![Dot Graph](images/dotgraph)

---

## 🔩 SolidWorks — 3D CAD Modelling & Flow Add-In

### Overview

SolidWorks was utilized for the **geometric modelling of the 61-meter Delta IV vehicle** and external aerodynamics testing using the Flow Simulation add-in.

![Delta IV Full Assembly — SolidWorks CAD](images/3dmodel)

### SolidWorks Flow Simulation Add-In

To ground the MATLAB optimization in reality, the CAD model was subjected to supersonic external flow conditions to extract accurate drag forces.

**Setup & Execution:**

- **Fluid:** Air (Compressible)
- **Boundary Conditions:** Computational domain configured to test specific flight points
- **Outputs Extracted:** Mach number contours across the payload fairing, pressure distributions, and velocity flow trajectories

**Validation:**

At a velocity of **680.539 m/s**, the simulation calculated a total aerodynamic drag force of **17,357,360.526 N**. This mapped to a $C_d$ of **0.363**, verifying the theoretical drag calculations used in the MATLAB trajectory arrays.

![SolidWorks External Flow — Rocket Flow Trajectories](images/rocketflow)

![SolidWorks Mach Contour](images/Mdot)

---

## 🌊 ANSYS Fluent — Computational Fluid Dynamics (CFD)

### Overview

ANSYS Fluent provides **high-fidelity CFD for the RS-68A engine nozzle**. The analysis captures the extreme thermodynamic gradients of cryogenic $LH_2$/LOX combustion expanding through a converging-diverging nozzle.

### Geometry & Mesh

- **Mesh Type:** 2D Axisymmetric structured mesh generated via ANSYS Meshing
- **Refinement Strategy:** Face meshing and edge sizing were strictly applied to the throat region to correctly capture the choked flow phenomenon and ensure accurate velocity choking.

![RS-68A Nozzle Mesh — 2D Axisymmetric](images/nozzlemeshing)

### Boundary Conditions & Solver Setup

| Setting | Value |
|---|---|
| Inlet | Pressure inlet at $P_0$ = 9.7 MPa, $T_0$ = 3600 K |
| Outlet | Pressure outlet at Sea Level (101.3 kPa) |
| Fluid | Ideal gas matched to steam-rich Hydrolox exhaust |

### Key Results

- **Velocity & Mach Contours:** The fluid smoothly accelerated through the throat, reaching an exhaust velocity exceeding **3716 m/s**.
- **Shock Structures:** The CFD successfully visualized the formation of **oblique shocks** outside the nozzle at sea-level atmospheric pressure, proving the nozzle design operates safely without dangerous internal flow separation.
- **Thermodynamics:** Temperature contours mapped the drop from **3600 K** in the chamber down to **240 K** in the expanded plume.

![Nozzle Mach Number Distribution](images/nozzlemach)

![Nozzle Velocity Contour](images/nozzlevelo)

![Nozzle Temperature Contour](images/nozzletemp)

![Nozzle Pressure Contour](images/pressurecont)

![Nozzle Sonic Line](images/sonicsont)

![Turbulence Contour](images/turbcont)

![Stagnation Temperature Contour](images/stagtemp)

![Stagnation Pressure Contour](images/stagoressure)

![Engine Nozzle Overview](images/engine)

![Results Graphs Summary](images/resultsgraphs)

![Rocket Results Overview](images/rocketresults)

![Rocket Results (Extended)](images/rocketresults2)

---

## 🔁 Simulink — Prototype Simulation via Dynamic Similarity

### Overview

A scale prototype of the rocket was mathematically evaluated using **dynamic similarity (dimensional analysis)**. Simulink was used to model the kinematic ascent of the prototype under matched non-dimensional conditions compared to the full-scale vehicle.

### Dimensional Analysis & Pi-Group Derivation

Using the Buckingham-Pi theorem, a geometric scale factor of $\lambda = 0.016$ was established — scaling the **62.5 m full-size rocket down to a 1.0 m, 0.08 m diameter model**.

Two distinct propulsion methodologies were analyzed for the scale model:

| Case | Propellant | Exit Velocity ($V_e$) | $I_{sp}$ |
|---|---|---|---|
| Case A — Cold-Gas | Compressed air (10 bar gauge) | 534.9 m/s | 54.5 s |
| Case B — Scaled Chemical | $LH_2$/LOX (RS-68A thermodynamics) | 3945.71 m/s | 402.2 s |

### Simulink Model Architecture

The Simulink environment mapped the discrete trajectory ODEs into a visual block diagram. Subsystems integrated thrust generation, variable gravity, and aerodynamic drag. Outputs logged include downrange distance (30° insertion angle trajectory), altitude, velocity, and mass.

![Simulink Canvas — Full Scale Reference Model](images/simulinkcanva1)

![Simulink Canvas — Prototype Model](images/simulinkcanva2)

### Simulation Results

![Prototype A — Simulation Graphs](images/simulationgraphsprototypeA)

![Prototype A — Trajectory Simulation](images/simulationprototypeA)

![Prototype B — Trajectory Simulation](images/simulationprototypeB)

![Case B Graph — Altitude & Velocity](images/caseBgraph)

### Validation

The Simulink plots demonstrated that while Mach similarity is automatically satisfied in free flight, Reynolds number similarity diverges. However, because the prototype's Reynolds number ($9.08 \times 10^5$) remains strictly in the **turbulent regime**, the boundary layer behaves consistently with the full-scale vehicle, confirming the validity of the scaled flight tests.

---

## 📊 Key Results & Validation

| Metric | MATLAB (Theoretical/Numerical) | ANSYS / SolidWorks (CFD) | Scale Model (Case B) |
|---|---|---|---|
| Target Apogee | 200,000.00 m ($\pm 0.328$ m) | — | 996.37 m |
| Mission Time | 300.0 s | — | 8.98 s |
| Max Drag Force | Computed via variable spline | 17.35 MN @ 680 m/s | 695.5 N |
| RS-68A Exit Velocity | 3945.7 m/s | ~3716.0 m/s | 3945.71 m/s |
| RS-68A Exit Mach | 5.97 | 5.97 | — |
| Max-Q (Dynamic Pressure) | 130.24 kPa @ 24.30 s | — | — |

![My Images — Results Collection](images/myimages)

---

## 🛠️ Dependencies & Software Versions

| Tool | Version | Purpose |
|---|---|---|
| MATLAB | R2023b+ | Optimization, RK4 trajectory, spline interpolation |
| SolidWorks | 2023+ | 3D CAD modelling, aerodynamic drag analysis |
| SolidWorks Flow Simulation | 2023+ | External flow supersonic CFD |
| ANSYS Fluent | 2026 R1 (Student) | Nozzle combustion thermodynamics, shock capture |
| ANSYS Meshing | 2026 R1 (Student) | 2D axisymmetric grid generation |
| Simulink | R2023b+ | Dynamic similarity kinematic simulation |

---

## 👩‍💻 Authors & Acknowledgements

**Simulation, Software Architecture & Report Compilation Lead:** 3li

**Physical Modelling, Dimensional Analysis & Theoretical Calculations — Team 13:**
Ahmed Ali Ahmed, Safey Eldeen Samy, Amr Waleed Bakr, Basel Ibrahim Ahmed, Ahmed Sayed Sobhi,
Mohamed Abdallah Abdelkader, Abdelmaqsoud Maher, Mahmoud Samy Abdelghafar, Ahmed Tamer Mahmoud,
Omar Mamdouh Mohamed, Pierre Ossama Magdy, Essam Mohamed Ghamry, Peter George Makram, and Mohamed Nabil Ali.

Developed as part of the **MEP212 S: Fluid Mechanics and Turbomachinery** project at:

**Ain Shams University — Faculty of Engineering, Mechatronics Department (Spring 2026)**

Supervised by **Prof. Ashraf Ghorab** and **Prof. Walid Aboelsoud**.

---

*For questions, issues, or collaboration, please open a GitHub Issue or reach out via the repository contact.*
