# Rollover Stability Control - Software Requirements Specification

**Version:** 1.1
**Date:** 2025-04-03

**1. Introduction**

* **1.1 Purpose:** This document specifies the requirements for the Rollover Stability Control software module for an Articulated Dump Truck (ADT). This module calculates stability limits to mitigate rollover risk.
* **1.2 Scope:** These requirements define the functionality, performance, interfaces, and constraints of the software module responsible for calculating the combined Center of Gravity (CG), lateral acceleration limit, and resultant speed limit. It defines *what* the software must do. Input acquisition and output usage details are treated as interface requirements.
* **1.3 Definitions and Acronyms:**
    * ADT: Articulated Dump Truck
    * CG: Center of Gravity
    * SDD: Software Design Document
    * SRS: Software Requirements Specification
    * FR: Functional Requirement
    * NFR: Non-Functional Requirement
    * ECU: Electronic Control Unit
    * CAN: Controller Area Network
    * m/s²: Meters per second squared
    * kph: Kilometers per hour
    * rad: Radians
    * deg: Degrees
    * mps: Meters per second
* **1.4 References:**
    * `RolloverStability_SDD_v1.1.md`: Software Design Document detailing the implementation approach.
    * `rollover_stability.c`: C source code implementation.
    * ADT System Specification Document (Hypothetical): Document defining overall vehicle system requirements.
    * ISO 26262 (Road vehicles – Functional safety) - *Relevant if developing to automotive safety standards*.

**2. Overall Description**

* **2.1 Product Perspective:** This software module is a component of the ADT's overall safety and control system, likely residing on an Electronic Control Unit (ECU). It receives vehicle state information and provides a calculated speed limit to potentially restrict the vehicle's maximum operating speed under certain conditions.
* **2.2 Product Functions Summary:**
    * Calculate the combined vertical CG of the ADT based on load.
    * Determine the maximum lateral acceleration the vehicle can withstand given the current CG, track width, and ground roll angle.
    * Calculate a recommended speed limit based on the lateral acceleration limit and the current articulation angle.
    * Handle specific conditions like zero load and zero articulation.
* **2.3 User Characteristics:** The end-users are the ADT operator (indirectly, through potentially limited speed) and service technicians (for diagnostics). The primary consumer of the output is another software module within the vehicle's control system.
* **2.4 System Context Diagram (Mermaid):**

    ```mermaid
    graph TD
        subgraph "External Systems"
            Sensors -->|Input Data (Roll, Load, Articulation)| RS_Module
            ECU_Control[Vehicle Control ECU]
            Operator([Operator]) -.-> |Indirectly Affected| ECU_Control
        end

        subgraph "Rollover Stability Module (RS_Module)"
             direction LR
             InputProcessing[Accept Inputs FR-006] --> CalculateCG[Calc CG FR-001, FR-005]
             CalculateCG --> CalculateLatLimit[Calc Lat Accel Limit FR-002]
             CalculateLatLimit --> CalculateSpeedLimit[Calc Speed Limit FR-003, FR-004]
             CalculateSpeedLimit --> OutputProcessing[Provide Outputs FR-007]
        end

         OutputProcessing -->|Calculated Speed Limit, Status| ECU_Control

    style Operator fill:#fff,stroke:#333,stroke-width:2px

    ```

* **2.5 Constraints:**
    * Must be implemented in C.
    * Must run on the target ADT ECU (specific hardware constraints TBD).
    * Must use provided machine parameters (Track Width, CG heights, etc.).
    * Calculation latency must be low enough for real-time control (see NFR-001).
* **2.6 Assumptions and Dependencies:**
    * Accurate input values (ground roll, weights, articulation angle) are provided.
    * Machine parameters configured in the software match the physical vehicle.
    * The underlying physics models used in the design (see SDD Section 8) are adequate.

**3. Specific Requirements**

* **3.1 Functional Requirements (FR):**
    * **FR-001:** The software **shall** calculate the combined vertical Center of Gravity (CG) height of the loaded machine. It shall use the unloaded machine weight, load weight, unloaded machine CG height, and the load's CG height as inputs. The calculation shall be based on a weighted average. *(Ref: SDD 4.1, `calculate_combined_cg`)*
    * **FR-002:** The software **shall** calculate the maximum allowable lateral acceleration limit in m/s². This calculation shall use the combined CG height, ground roll angle, vehicle track width, and the acceleration due to gravity (9.81 m/s²). The calculation shall account for moments tending to cause rollover versus stabilizing moments. The software shall detect and flag conditions of static instability (rollover risk even with zero lateral acceleration). *(Ref: SDD 4.2, `calculate_lateral_accel_limit`)*
    * **FR-003:** The software **shall** calculate a recommended maximum speed limit in kph. This calculation shall use the calculated lateral acceleration limit and the current articulation angle. The calculation shall relate lateral acceleration to speed and turn radius (`a = v^2/R`), using a simplified model for turn radius based on articulation angle and a characteristic vehicle length. *(Ref: SDD 4.4, `calculate_speed_limit_mps`)*
    * **FR-004:** The software **shall** handle a zero (or near-zero) articulation angle. In this condition (driving straight), the speed limit calculation shall not be constrained by turning dynamics, although the limit determined by ground roll (FR-002) must still be respected. *(Ref: SDD 4.3, SDD 4.4, `calculate_turn_radius`, `calculate_speed_limit_mps`)*
    * **FR-005:** The software **shall** handle a zero (or near-zero) load weight. In this condition, the combined CG calculation (FR-001) shall result in the unloaded machine's CG height. *(Ref: SDD 4.1, `calculate_combined_cg`)*
    * **FR-006:** The software **shall** accept the following inputs:
        * Ground roll angle (degrees)
        * Unloaded machine weight (kg)
        * Load weight in bin (kg)
        * Articulation angle (degrees) *(Ref: SDD 5.2, `MachineInputs` struct)*
    * **FR-007:** The software **shall** output the calculated speed limit (kph). It may also output the calculated lateral acceleration limit (m/s²) and a static instability status flag for diagnostic or informational purposes. *(Ref: SDD 5.2, `StabilityLimits` struct)*
* **3.2 Non-Functional Requirements (NFR):**
    * **NFR-001 (Performance):** The entire calculation cycle (from input reception to speed limit output) **shall** complete within 50 milliseconds [Value TBD based on system needs].
    * **NFR-002 (Reliability):** The software **shall** handle potential floating-point exceptions (e.g., division by zero) gracefully, returning safe/default values (e.g., zero speed limit) where appropriate. *(Partially addressed in SDD error handling)*
    * **NFR-003 (Safety):** The software is considered safety-related. Development practices should align with relevant standards (e.g., MISRA C, potentially ISO 26262 ASIL level TBD). Output values must be conservative in cases of uncertainty or error.
    * **NFR-004 (Maintainability):** The code **shall** be well-commented, adhering to defined coding standards. Constants representing machine parameters shall be clearly defined and easily configurable. *(Partially addressed in code comments, SDD references)*
    * **NFR-005 (Portability):** The code **shall** be written in standard C (e.g., C99 or C11) with minimal platform-specific dependencies, primarily relying on `math.h`.
* **3.3 Interface Requirements:**
    * **IF-001:** The software module **shall** receive input data (FR-006) through defined function calls or a shared memory interface as specified by the overall ECU architecture. Data types and units shall match FR-006.
    * **IF-002:** The software module **shall** provide output data (FR-007) through defined function return values, shared memory, or CAN messages as specified by the overall ECU architecture. Data types and units shall match FR-007.
