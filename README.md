# Autonomous 4WD Vehicle Platform: System Identification & Discrete PI Control

An experimental autonomous 4WD ground vehicle platform designed in collaboration with **John Deere** for strict path-tracking trajectories within a 10x10m operative perimeter. The project transitions away from traditional trial-and-error embedded tuning by deploying formal mathematical **Dynamic Plant Identification (ARX Modeling)** in MATLAB, paired with localized, multi-loop discrete PI controllers implemented on an **STM32F103C8T6 micro-controller**.

## 📊 Project Metadata
* **Academic Timeline:** Undergraduate Project (November 2023)
* **Corporate Collaborator:** John Deere (Hardware & Architecture Guidelines)
* **Engineering Focus:** Dynamic System Identification, State-Space Estimation, Multivariable Control Systems, Low-Level Firmware Construction.

---

## 🛠️ Mechatronic System Architecture

The robot employs a robust, highly modular hardware topology optimized for deterministic signal management and precise execution response:

* **Processing Unit & Peripherals:** Managed entirely via an STM32 Bluepill MCU. Features dynamic clock configuration feeding 4 independent high-resolution 12-bit Timer-based PWM signals—yielding control increments as fine as 0.8mV per motor drive line—and 4 dedicated hardware external interrupts (EXTI) for speed tracking.
* **Actuation & Kinematics:** Driven by a 48:1 gearmotor layout controlled through L298N H-Bridges executing *skid-steer* zero-radius turning configurations.
* **Sensor Fusion & Signal Paths:** * **MPU6050 6-DOF IMU:** Communicates over an I2C bus at 100 kHz. The Z-axis gyroscope data is filtered via a high-pass digital architecture and integrated every 50ms inside a hardware timer callback to extract the relative yaw angle.
  * **H206 Optical Encoders:** Configured over pulldown hardware lines to measure real-time wheel Revolutions Per Minute (RPM).
  * **QMC5883L Magnetometer:** Calibrated through a normalized 3D point-cloud sphere in MATLAB to mitigate local hard/soft iron distortions (documented and analyzed under environmental interference constraints).

---

## 📐 Control Theory Framework & System Identification

Rather than assuming ideal linear motor parameters, the system's differential dynamics were rigorously audited through three distinct analytical testing paradigms in MATLAB to suppress inter-axial wheel drift:

1. **Wide-Pulse Modeling:** Evaluated transient variations across midpoint boundaries.
2. **PRBS (Pseudo-Random Binary Sequence):** Deployed a multi-frequency stochastic binary input pattern to stress-test high-frequency variations across the plants.
3. **Deterministic Step Response Analysis (Selected):** Modeled utilizing a mathematical Autoregressive with Exogenous Input (**ARX**) matrix structure ($Q = X'X$). This framework achieved a dynamic model resemblance exceeding 70%, capturing authentic motor back-EMF and mechanical friction profiles.

The resulting gains were mapped into a **Dual-Layer Cascaded Control Structure**:
* **Inner Loop:** 4 individual Proportional-Integral (PI) discrete controllers executing inside a deterministic TIM2 period to force equal rotational speeds across all wheels, neutralizing unexpected physical slipping.
* **Outer Loop:** A master directional PI controller that continuously evaluates the integrated gyroscope telemetry to control the precise trajectory angles.

---

## 🔍 Post-Years Engineering Evaluation & Retrospective

Reviewing this architecture from a post-graduate standpoint highlights critical mechatronic challenges that serve as premier case studies for production improvements:

### 1. High-Frequency Noise Isolation: STM32 vs. Slower Architectures
* **The Vulnerability:** Initial testing with cheap FC-03/LM393 encoder modules introduced immense signal noise, creating phantom high-RPM spikes that destabilized the derivative loops. 
* **The Insight:** Slower platforms (e.g., Arduino Uno) mask these flaws because their low clock cycles act as an implicit low-pass hardware filter. The high-performance clock of the STM32 registered every micro-spike. The resolution required replacing the comparators with rugged H206 hardware sensors and implementing a digital moving average filter in code.

### 2. Structural Dynamics & Structural Insulation Trade-offs
* **The Vulnerability:** To prevent mechanical tire slip on heavy torque demands, the chassis was upgraded from acrylic to a custom 1/8-inch steel plate cut via a CNC plasma cutter. However, mechanical vibration caused the underlying H-bridge pins to pierce the protective insulation tape, short-circuiting the microcontroller.
* **The Insight:** Prototyping structural updates requires strict adherence to electronic isolation housing. While the system was reverted to acrylic plates due to project time constraints, a modern iteration would implement a hybrid sand-blasted aluminum skeleton with isolated 3D-printed PETG brackets to isolate signal paths while maintaining structural mass.

### 3. Sensor Selection and Telemetry Drift
* **The Vulnerability:** Fusing magnetometer data with the gyroscope was intended to resolve yaw drift, but local magnetic fields (steel building columns) severely distorted the 3D point cloud calibration.
* **The Insight:** Over short execution windows, the gyroscope's standalone relative integration proved highly precise. For automotive or agricultural scaling, the magnetometer would be isolated on an elevated structural mast or replaced by a real-time kinematics (RTK) differential GNSS module.
