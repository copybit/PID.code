# PIDPython

Python implementation of a reusable PID controller plus a collection of simulation examples that reproduce typical control-engineering scenarios such as manual/automatic switching, setpoint weighting, anti-windup, filtering, feedforward compensation, gain scheduling, and selector-based control.

## Overview

This repository is organized around a single controller class, [`PID.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/PID.py), and a set of standalone scripts that simulate different closed-loop control problems.

The project is not a general application with a CLI. Instead, it is a teaching and experimentation codebase:

- `PID.py` implements the controller logic.
- `process_and_simulation_data.py` builds the initial conditions, process parameters, arrays, disturbance/noise signals, and reference trajectories for each example.
- `show_graphical_results.py` generates the plots for each scenario.
- `Example*.py` scripts run individual simulations.

The examples appear to be a Python port of MATLAB material from a paper/tutorial on practical PID implementation.

## Main Features

- Discrete-time PID controller in incremental/combined form.
- Manual, automatic, and tracking modes.
- Setpoint weighting through parameters `b` and `c`.
- Output saturation limits and rate-of-change limits.
- Back-calculation anti-windup through `Tt`.
- Optional feedforward contribution inside the controller call.
- Second-order measurement filter via `filter()`.
- Easy cloning of controller settings with `copy()`.

## Repository Structure

- [`PID.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/PID.py): PID controller dataclass and filter implementation.
- [`process_and_simulation_data.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/process_and_simulation_data.py): per-example plant models, buffers, noise, disturbances, and constants.
- [`show_graphical_results.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/show_graphical_results.py): plotting logic for all examples.
- [`tightfig.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/tightfig.py): thin helper around `matplotlib.pyplot.tight_layout()`.
- [`Example_basic.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example_basic.py): basic loop with measurement filtering and noisy output.
- [`Example1_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example1_paper.py): manual-to-auto switching.
- [`Example2_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example2_paper.py): PI/P switching and rate-limit-ready setup.
- [`Example3_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example3_paper.py): setpoint weighting comparison (`b = 1`, `0.5`, `0`).
- [`Example4_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example4_paper.py): feedforward compensation and anti-windup strategies.
- [`Example5_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example5_paper.py): noise filtering with different `Tf` values.
- [`Example6_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example6_paper.py): gain scheduling for nonlinear tank level control.
- [`Example7_paper.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example7_paper.py): two-controller selector / override control.
- `Example*_asv.py`: thin wrappers that simply call the corresponding main example.
- [`Example_basic_greenblue.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/Example_basic_greenblue.py): same basic example with custom plot colors.
- [`semilla.mat`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/semilla.mat): MATLAB RNG seed used to reproduce the noise sequence.
- [`v.mat`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/v.mat): data used by Example 7.
- [`compare_against_matlab.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/compare_against_matlab.py): helper script intended to compare Python results against MATLAB exports.

## Installation

Create an environment and install the required packages:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Dependencies:

- `numpy`
- `scipy`
- `matplotlib`

## Running the Examples

Run any example directly:

```bash
python3 Example_basic.py
python3 Example1_paper.py
python3 Example4_paper.py
python3 Example6_paper.py
```

Each script:

1. Loads a predefined scenario with `load_process_and_simulation_data()`.
2. Configures one or more `PID` instances.
3. Simulates the plant in a time loop.
4. Stores signals in arrays.
5. Plots the results with `show_graphical_results()`.

If you are running in a headless environment, use:

```bash
MPLBACKEND=Agg python3 Example_basic.py
```

In that case, Matplotlib may warn that `plt.show()` is non-interactive, which is expected.

## PID Controller API

The controller is defined as:

```python
from PID import PID

pid = PID()
```

Main tunable parameters:

- `kp`, `ki`, `kd`: proportional, integral, and derivative gains.
- `b`, `c`: setpoint weighting factors.
- `Dt`: sample time.
- `Tf`: filter time constant used by `filter()`.
- `Tt`: tracking time constant used for back-calculation anti-windup.
- `minlim`, `maxlim`: output limits.
- `dumin`, `dumax`: rate-of-change limits on the control signal.
- `u0`: nominal bias/control offset.

Important methods:

- `init(r0=0.0, y0=0.0, u0=None, uff0=0.0)`: initializes controller states.
- `control(r, y, uff=0.0, uman=0.0, utrack=0.0, mode="AUTO")`: computes the next control action.
- `filter(y)`: applies a second-order low-pass filter to the measured signal.
- `copy()`: deep-copies the controller, useful for comparative simulations.

Supported modes in `control()`:

- `AUTO`: normal closed-loop PID operation.
- `MAN`: manual mode, output is forced to `uman`.
- `TRACK`: tracking mode, internal state follows `utrack` for bumpless transfer and selector schemes.

## Example Summary

- `Example_basic.py`: baseline noisy first-order-plus-dead-time control loop using the built-in filter.
- `Example1_paper.py`: demonstrates bumpless transfer from manual operation to automatic control.
- `Example2_paper.py`: shows how the same class can behave as PI or P/PD-like control when integral action is disabled.
- `Example3_paper.py`: compares the effect of different setpoint weights on response smoothness and control effort.
- `Example4_paper.py`: compares no saturation, external saturation only, clamping, and back-calculation; also adds a feedforward compensator for a measurable disturbance.
- `Example5_paper.py`: studies measurement filtering under output noise.
- `Example6_paper.py`: compares two gain-scheduling implementations for a nonlinear tank process:
  parallel controllers with tracking, and a single controller whose gains are updated online.
- `Example7_paper.py`: implements selector/override logic between two controllers, with the inactive controller kept synchronized through tracking.

## Data and Models

Most examples use a discrete first-order-plus-dead-time process model:

- gain `k`
- time constant `t`
- delay `l`

The process is simulated with coefficients `ap` and `bp`, computed from the sample time `dt`.

Special cases:

- Examples 0, 5: add white measurement noise.
- Example 4: includes a disturbance model and a discrete feedforward compensator.
- Example 6: uses a nonlinear tank-level model.
- Example 7: uses preloaded signal `v` from `v.mat`.

## Notes About MATLAB Comparison

[`compare_against_matlab.py`](/Users/joseluisguzmansanchez/Documents/MATLAB/Tore/PID_implementation/PIDPython/compare_against_matlab.py) is designed to compare Python results against MATLAB-generated `.mat` snapshots. However, in this repository snapshot the MATLAB companion files referenced by that script, such as `Example_basic.m` or `export_matlab_workspace.m`, are not present. That means the comparison workflow is only partially included here.

## Practical Notes

- The plotting helpers expect an interactive Matplotlib backend when used normally.
- On some systems, Matplotlib may warn that its default cache directory is not writable. Setting `MPLCONFIGDIR` to a writable folder resolves it.
- The project has no packaging metadata yet, so it is best used as a script-based repository.

Example:

```bash
mkdir -p /tmp/mplconfig
MPLCONFIGDIR=/tmp/mplconfig python3 Example5_paper.py
```

## Verified Behavior

The following scripts were smoke-tested successfully in a headless environment:

- `Example_basic.py`
- `Example4_paper.py`
- `Example6_paper.py`

## License

No license file is included in the current repository snapshot. Add one before redistributing the code.
