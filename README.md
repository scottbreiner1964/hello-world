# Rollover Stability Control - Software Design Document

**Version:** 1.1
**Date:** 2025-04-03

**1. Introduction**

* **1.1 Purpose:** This document describes the design of the software module responsible for calculating rollover stability limits for an Articulated Dump Truck (ADT). It details the architecture, algorithms, interfaces, and data structures used to determine the combined Center of Gravity (CG), maximum lateral acceleration limit, and resulting speed limit based on articulation.
* **1.2 Scope:** This document covers the design of the calculation logic implemented in `rollover_stability.c`. It assumes inputs (ground roll, weights, articulation) are provided by external systems (sensors, CAN bus interface) and the output (speed limit) is consumed by another system (e.g., vehicle control unit). Input validation and sensor fusion are outside the scope of this specific calculation module's design details but are acknowledged as necessary interfaces.
* **1.3 Definitions and Acronyms:** (Same as SRS Section 1.3)
* **1.4 References:**
    * `RolloverStability_SRS_v1.1.md`: Software Requirements Specification for this module.
    * `rollover_stability.c`: C source code implementation.
    * Vehicle Dynamics Textbooks (e.g., Milliken & Milliken - Race Car Vehicle Dynamics, Gillespie - Fundamentals of Vehicle Dynamics) - *For underlying physics principles*.
    * ADT Specific Datasheet - *Required for accurate machine parameters (`TRACK_WIDTH`, `UNLOADED_CG_HEIGHT`, etc.)*.

**2. System Overview**

* The Rollover Stability module is a computational component designed to run on an embedded ECU within the ADT. Its primary goal is to enhance safety by calculating a dynamic speed limit that mitigates the risk of rollover caused by excessive lateral acceleration, ground slope, and vehicle articulation. It receives real-time or near-real-time inputs, performs physics-based calculations, and outputs a recommended maximum speed.

**3. Architectural Design**

* **3.1 Overview:** The software follows a modular, functional design. It consists of helper functions for unit conversions and core calculation functions for CG, lateral acceleration limit, and speed limit.
* **3.2 Components:**
    * **Input Interface (Conceptual):** Receives `MachineInputs` data (assumed to be populated externally).
    * **Unit Conversion:** Helper functions (`deg_to_rad`, `rad_to_deg`, `mps_to_kph`).
    * **CG Calculation Module:** `calculate_combined_cg()` function.
    * **Lateral Limit Calculation Module:** `calculate_lateral_accel_limit()` function.
    * **Turn Radius Calculation Module:** `calculate_turn_radius()` function (helper for speed limit).
    * **Speed Limit Calculation Module:** `calculate_speed_limit_mps()` function.
    * **Output Interface (Conceptual):** Provides `StabilityLimits` data (specifically `speed_limit_kph` and potentially `lateral_accel_limit_mps2` or status flags) to other ECU components.
* **3.3 Data Flow Diagram (Mermaid):**

    ```mermaid
      flowchart TD
          A[Christmas] -->|Get money| B(Go shopping)
          B --> C{Let me think}
          C -->|One| D[Laptop]
          C -->|Two| E[iPhone]
          C -->|Three| F[fa:fa-car Car]
  
    ```

**4. Detailed Design**

* **4.1 `calculate_combined_cg` (`rollover_stability.c`)**
    * **Functionality:** Calculates the weighted average vertical height of the combined machine and load CG.
    * **Algorithm:** `h_cg_combined = (W_machine * h_chassis + W_load * h_load) / (W_machine + W_load)`.
    * **Inputs:** `unloaded_weight` (kg), `load_weight` (kg), `unloaded_cg_h` (m), `load_cg_h` (m).
    * **Outputs:** Combined CG height (m).
    * **Error Handling:** Returns `unloaded_cg_h` if `load_weight` <= 0 or `unloaded_weight` <= 0.
    * **Traceability:** FR-001, FR-005.
* **4.2 `calculate_lateral_accel_limit` (`rollover_stability.c`)**
    * **Functionality:** Calculates the maximum lateral acceleration based on moment balance around the low-side wheel contact point.
    * **Algorithm:** `a_lat_limit = g * [ (cos(roll) * track_width / (2 * h_cg)) - sin(roll) ]`. Includes check for static instability (`tan(roll) * h_cg >= track_width / 2` or if the formula yields < 0).
    * **Inputs:** `combined_cg_h` (m), `ground_roll_rad` (rad), `track_width` (m).
    * **Outputs:** Lateral acceleration limit (m/s²). Returns -1.0 to indicate static instability.
    * **Error Handling:** Returns 0.0 if `combined_cg_h` or `track_width` are non-positive or near zero. Explicitly checks for and flags static instability.
    * **Traceability:** FR-002.
* **4.3 `calculate_turn_radius` (`rollover_stability.c`)**
    * **Functionality:** Provides a simplified estimation of the turn radius.
    * **Algorithm:** `R = L / tan(articulation_rad)`.
    * **Inputs:** `articulation_rad` (rad), `characteristic_length` (m).
    * **Outputs:** Turn radius (m).
    * **Error Handling:** Returns `INFINITY` if `articulation_rad` is near zero or `characteristic_length` is non-positive.
    * **Traceability:** Supports FR-003, FR-004.
* **4.4 `calculate_speed_limit_mps` (`rollover_stability.c`)**
    * **Functionality:** Calculates the maximum permissible speed based on the lateral acceleration limit and turn radius.
    * **Algorithm:** `v_limit = sqrt(a_lat_limit * R)`. Uses `calculate_turn_radius`.
    * **Inputs:** `lat_accel_limit` (m/s²), `articulation_rad` (rad), `characteristic_length` (m).
    * **Outputs:** Speed limit (m/s).
    * **Error Handling:** Returns 0.0 if `lat_accel_limit` is non-positive. Returns a high speed (capped later) if turn radius is infinite (straight driving).
    * **Traceability:** FR-003, FR-004.
* **4.5 `main` Function (`rollover_stability.c`)**
    * **Functionality:** Orchestrates the calculation flow, acquires example inputs, calls calculation functions, performs final checks (static instability override, max speed cap), and prints results.
    * **Traceability:** Demonstrates FR-006 (input usage), FR-007 (output formatting).

**5. Interface Design**

* **5.1 Internal Interfaces:** Functions call each other as described in Section 4. Data is passed via function arguments and return values.
* **5.2 External Interfaces (Conceptual):**
    * **Input:** The module expects a structure `MachineInputs` populated with values in specified units (degrees, kg). Source: Sensor fusion module / CAN interface.
    * **Output:** The module produces a structure `StabilityLimits`. The primary output `speed_limit_kph` is intended for consumption by a vehicle speed control system. `lateral_accel_limit_mps2` and `statically_unstable` flag may be used for diagnostics or driver information. Destination: Vehicle Control Unit / Display Module via ECU internal communication or CAN.

**6. Data Design**

* **6.1 `MachineInputs` Struct:** Contains input parameters (double precision). See `rollover_stability.c`.
* **6.2 `StabilityLimits` Struct:** Contains calculated output values (double precision) and status flag (boolean). See `rollover_stability.c`.
* **6.3 Constants:** Defined using `#define` for physical constants (`GRAVITY`, `PI`) and machine parameters (`TRACK_WIDTH`, etc.). Machine parameters *must* be configured accurately.

**7. Requirements Traceability**

* See `Traceability:` entries in Section 4 and `ref` tags in `rollover_stability.c` comments. The diagrams in SRS 2.4 and SDD 3.3 also visually link requirements to components/flows. A separate traceability matrix is recommended for larger projects.

**8. Assumptions and Dependencies**

* **Assumptions:**
    * Physics models (CG averaging, moment balance, turn radius) are sufficiently accurate for this safety application.
    * Input data (weights, angles) is accurate and provided in real-time or near real-time.
    * Machine parameters (`TRACK_WIDTH`, `UNLOADED_CG_HEIGHT`, `LOAD_BIN_CG_HEIGHT`, `CHARACTERISTIC_LENGTH_L`) are correctly defined for the specific ADT model.
    * Load CG height (`LOAD_BIN_CG_HEIGHT`) is a reasonable estimate; actual height can vary.
    * Tire deformation and suspension dynamics are neglected in this simplified model.
* **Dependencies:**
    * Requires accurate sensor inputs (inclinometer, load cells, articulation sensor).
    * Depends on correctly defined machine parameters.
    * Requires `math.h` library functions.
    * Target platform: Embedded controller capable of running compiled C code with floating-point arithmetic.
