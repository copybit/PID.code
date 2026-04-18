# PID.code

This repository contains a unified MATLAB and Python implementation of a reusable, feature-complete PID controller, together with a collection of simulation examples that reproduce common industrial and academic control scenarios.

The project is intended as a **teaching, experimentation, and reference codebase** for practical PID implementation, covering both fundamental and advanced control strategies.

---

## 📦 Repository Structure

```text
.
├── matlab/        % MATLAB implementation and examples
├── python/        % Python implementation and examples
└── README.md      % This file
```

Each language folder is self-contained and includes:

* A reusable PID controller implementation
* Simulation scripts for different control scenarios
* Shared utilities for process definition and plotting

---

## 🎯 Project Goals

This repository is not a packaged library or application. Instead, it focuses on:

* Providing a **clear and reusable PID implementation**
* Demonstrating **practical control engineering techniques**
* Allowing **side-by-side comparison between MATLAB and Python implementations**
* Serving as a **didactic resource** for students and practitioners

---

## ⚙️ Controller Features

Both implementations (MATLAB and Python) support:

* P, PI, PD, and PID control
* Manual, automatic, and tracking modes
* Setpoint weighting (`b`, `c`)
* Feedforward integration
* Output saturation and rate limiting
* Anti-windup via back-calculation
* Second-order measurement filtering
* Controller duplication (copy/clone functionality)

---

## 🧪 Example Scenarios

The included examples cover typical control engineering situations:

* Manual to automatic transfer (bumpless switching)
* P / PD operation via integral deactivation
* Setpoint weighting effects
* Feedforward compensation
* Anti-windup strategies
* Measurement noise filtering
* Gain scheduling (linear and nonlinear systems)
* Selector / override control

Each example follows a consistent structure:

1. Load process and simulation data
2. Configure controller(s)
3. Run a discrete-time simulation loop
4. Store signals
5. Plot results

---

## 🔁 MATLAB vs Python

Both implementations are conceptually equivalent and designed to mirror each other:

| Aspect             | MATLAB                          | Python                           |
| ------------------ | ------------------------------- | -------------------------------- |
| Controller         | `@PID/PID.m` class              | `PID.py` class                   |
| Simulation scripts | `.m` scripts                    | `.py` scripts                    |
| Data setup         | `process_and_simulation_data.m` | `process_and_simulation_data.py` |
| Plotting           | MATLAB plotting                 | `matplotlib`                     |

This makes the repository useful for:

* Translating control logic between environments
* Teaching implementation differences
* Verifying numerical consistency

---

## 🚀 Getting Started

### MATLAB

1. Open MATLAB
2. Navigate to the `matlab` folder
3. Run any example:

```matlab
Example_basic
```

---

### Python

1. Create and activate a virtual environment
2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Run an example:

```bash
python3 Example_basic.py
```

---

## 🧠 Recommended Order

If you are new to the repository, a sensible progression is:

1. Basic example
2. MAN/AUTO transfer
3. Setpoint weighting
4. Feedforward and anti-windup
5. Gain scheduling
6. Selector control

---

## ⚠️ Notes

* This repository is script-based (not a packaged library)
* No license file is currently included
* MATLAB and Python implementations are maintained in parallel but independently
* Some auxiliary files (e.g., `.mat`, autosave files) are used for reproducibility

---

## 📚 Intended Use

This repository is especially useful for:

* Control engineering courses
* Research prototyping
* Algorithm validation
* Cross-language implementation comparison

---

## 🤝 Contributing

Contributions are welcome, especially:

* Improvements to numerical consistency between implementations
* Additional control scenarios
* Documentation enhancements

---

## 📄 License

No license has been defined yet. Add one before redistribution.

