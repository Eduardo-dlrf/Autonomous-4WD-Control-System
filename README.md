# Autonomous 4WD Vehicle Platform: System Identification & Cascaded PI Control

![Collaborator](https://img.shields.io/badge/Industry%20Collaborator-John%20Deere-006227?style=flat-square&logo=John-Deere&logoColor=FFDE00)
![Platform](https://img.shields.io/badge/Platform-STM32--HAL-blue?style=flat-square&logo=stmicroelectronics)
![Control](https://img.shields.io/badge/Control-Cascaded%20PI%20Loop-red?style=flat-square)
![Math](https://img.shields.io/badge/Analysis-MATLAB%20System%20ID-orange?style=flat-square&logo=mathworks)

An experimental autonomous 4WD ground vehicle platform developed as a technical challenge in collaboration with **John Deere**. The project achieves precise path-tracking trajectories within a closed perimeter by moving away from trial-and-error embedded tuning. Instead, it deploys formal mathematical **Dynamic Plant Identification (ARX Modeling)** in MATLAB, paired with localized, multi-loop discrete PI controllers implemented on an **STM32F103C8T6 micro-controller**.

## 📊 Project Metadata
* **Timeline:** Undergraduate Project - Industrial Automation Class (November 2023)
* **Engineering Discipline:** Mechatronics Engineering
* **Core Focus:** Dynamic System Identification, State-Space Estimation, Multivariable Control Systems, Low-Level Firmware Construction.

---

## 📁 Repository Structure

The codebase and engineering documentation are structured as follows:

```text
├── Core/                      # Native STM32 firmware framework
│   ├── Inc/                   # Architecture headers (.h) [PIDpwm.h, Directions.h]
│   └── Src/                   # Low-level application files (.c)
│       ├── main.c             # System initializations, state-machine execution, and ISRs
│       ├── PIDpwm.c           # Mathematical implementation of the discrete PI control algorithm
│       └── Directions.c       # Low-level H-Bridge GPIO motor activation mappings
├── Identification_MATLAB/     # Plant modeling arrays and mathematical analysis
│   ├── ARX_Escalon.pdf        # System identification via step-response matrices
│   ├── ARX_PRBS.pdf           # Stochastic validation tracking using Pseudo-Random Binary Sequences
│   └── ARX_E1.pdf             # Model cross-validation and transfer function extractions
├── Documents/                 # Formal technical documentation
│   └── Reporte_Automatizacion_Industrial.pdf # Comprehensive engineering report
└── Carritov3.ioc              # Graphical peripheral clock and pinout setup configuration
```
## 📐 Control Theory Framework & System Identification

Rather than assuming ideal linear motor parameters, the system's differential dynamics were rigorously audited through mathematical **Autoregressive with Exogenous Input (ARX)** matrix structures ($Q = X'X$) inside MATLAB to suppress inter-axial wheel drift:

* **Stochastic Testing (PRBS):** Deployed a multi-frequency pseudo-random binary sequence to stress-test high-frequency variations across the plants.
* **Step Response Analysis:** Selected for the final implementation due to a dynamic model resemblance exceeding 70%, successfully capturing authentic motor back-EMF and mechanical friction profiles.

The resulting gains were mapped into a **Dual-Layer Cascaded Control Structure**:
* **Inner Loop:** 4 individual Proportional-Integral (PI) discrete controllers executing inside a deterministic 250ms TIM2 period to force equal rotational speeds across all wheels, neutralizing unexpected physical slipping.
* **Outer Loop:** A master directional controller executing on a TIM4 50ms interval that continuously evaluates the integrated high-pass filtered gyroscope telemetry to modulate precise trajectory adjustments.

---

## ⚙️ Development Environment & Toolchain

* **IDE:** STM32CubeIDE v1.13 Utilizing the GCC ARM Embedded Toolchain.
* **Hardware Configuration Tool:** STM32CubeMX Embedded Core.
* **Low-Level Software Layer:** STM32F1xx HAL Peripherals Cores.
* **Analysis Subsystems:** MATLAB R2023a (Control System Toolbox & System Identification Toolbox).

---

## 🔍 Engineering Retrospective & Future Work (Post-Years Evaluation)

Looking back at this platform from a post-graduate perspective, the architecture stands as a solid foundation for multivariable systems, but it reveals clear constraints when compared against production-grade automotive standards. To preserve the historical integrity of this undergraduate milestone, **the codebase remains completely unaltered**, but the following architectural areas of opportunity have been identified for future industrial iterations:

### 1. High-Frequency Noise Isolation: STM32 vs. Slower Architectures
* **The Vulnerability:** Initial testing with cheap LM393 comparator encoder modules introduced immense signal noise, creating phantom high-RPM spikes that destabilized the control loops.
* **The Insight:** Slower processing platforms (e.g., Arduino Uno) inherently mask these flaws because their low clock cycles act as an implicit low-pass filter. The high-performance 72MHz clock of the STM32 registered every micro-spike. The resolution required replacing the comparators with rugged H206 hardware sensors and implementing a digital moving average filter in code.

### 2. Structural Dynamics & Electrical Insulation Trade-offs
* **The Vulnerability:** To prevent mechanical tire slip under heavy torque demands, the chassis was upgraded from acrylic to a custom 1/8-inch steel plate cut via a CNC plasma cutter. However, mechanical vibration caused the underlying H-bridge pins to pierce the protective insulation tape, short-circuiting the microcontroller.
* **The Insight:** Prototyping structural updates requires strict adherence to electronic isolation housing. While the system was reverted to acrylic plates due to project time constraints, a modern automotive iteration would implement a hybrid sand-blasted aluminum skeleton with isolated 3D-printed PETG brackets to isolate signal paths while maintaining optimal structural mass.

### 3. Sensor Selection and Environmental Telemetry Drift
* **The Vulnerability:** Fusing magnetometer data with the gyroscope was intended to resolve yaw drift, but local magnetic fields (such as steel building columns) severely distorted the 3D point-cloud calibration.
* **The Insight:** Over short execution windows, the gyroscope's standalone relative integration proved highly precise, leading to the deliberate deactivation of the magnetometer to optimize computational overhead. For automotive or agricultural scaling, the magnetometer would be isolated on an elevated structural mast or replaced by a real-time kinematics (RTK) differential GNSS module.
