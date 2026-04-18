# PID Implementation in MATLAB

This repository contains a MATLAB implementation of a discrete-time PID controller packaged as a class, plus a set of simulation examples that reproduce common industrial control scenarios.

The project is centered around the `PID` class located in `@PID/PID.m`. It supports:

- PI, PID, P, and PD operation
- manual, automatic, and tracking modes
- setpoint weighting
- feedforward integration
- output saturation
- rate limiting
- anti-windup by back-calculation
- second-order measurement filtering
- controller duplication through MATLAB handle copying

The accompanying example scripts show how to use the controller in different control engineering situations such as MAN/AUTO transfer, gain scheduling, selector override, disturbance feedforward, and noise filtering.

## Repository Structure

```text
.
├── @PID/PID.m                     % Main PID controller class
├── Example_basic.m                % Minimal closed-loop simulation example
├── Example1_paper.m               % MAN/AUTO transfer
├── Example2_paper.m               % P/PD operation and rate limiting
├── Example3_paper.m               % Setpoint weighting
├── Example4_paper.m               % Feedforward and anti-windup
├── Example5_paper.m               % Measurement-noise filtering
├── Example6_paper.m               % Gain scheduling for a tank level system
├── Example7_paper.m               % Selector/override control
├── process_and_simulation_data.m  % Shared process and simulation setup
├── show_graphical_results.m       % Shared plotting routine
├── tightfig.m                     % Figure layout helper
├── semilla.mat                    % Random seed for repeatable noise
├── v.mat                          % Disturbance profile used by Example 7
└── *.asv                          % MATLAB autosave files
```

## What the Project Does

The repository is not a single application. Instead, it is a reusable control component plus a collection of simulation scripts.

Each example follows the same overall pattern:

1. Select an `exampleID`.
2. Load the process model and simulation settings through `process_and_simulation_data.m`.
3. Create one or more `PID` objects.
4. Tune their parameters and initialize their internal states.
5. Run a discrete-time simulation loop.
6. Plot the results through `show_graphical_results.m`.

Most process examples are based on first-order-plus-dead-time models discretized with:

- `ap = exp(-Dt/T)`
- `bp = K*(1-ap)`
- `Lp = L/Dt`

where `K` is the static gain, `T` is the time constant, `L` is the time delay, and `Dt` is the sampling time.

## PID Class Overview

The `PID` class is implemented as a MATLAB handle class with copy support:

```matlab
classdef PID < handle & matlab.mixin.Copyable
```

This makes it easy to define a base controller and clone it when only a few parameters need to change, as done in several examples.

### Main Public Properties

Controller gains and weighting:

- `kp`, `ki`, `kd`: proportional, integral, and derivative gains
- `b`: setpoint weight for the proportional term
- `c`: setpoint weight for the derivative term
- `u0`: bias term used in position-form operation

Timing:

- `Dt`: sampling time
- `Tf`: time constant of the built-in second-order filter
- `Tt`: tracking time constant for anti-windup back-calculation

Constraints:

- `minlim`, `maxlim`: output saturation limits
- `dumin`, `dumax`: output rate limits

### Main Methods

#### `init(obj, r0, y0, u0, uff0)`

Initializes internal controller and filter states. This should be called before starting a simulation or after a reset.

#### `u = control(obj, r, y, uff, uman, utrack, mode)`

Computes the controller output.

Inputs:

- `r`: reference
- `y`: measured or filtered process output
- `uff`: feedforward contribution
- `uman`: manual output value when `mode == "MAN"`
- `utrack`: tracked signal when `mode == "TRACK"`
- `mode`: controller operating mode

Supported operating modes:

- `"MAN"`: the controller returns the manual command `uman`
- `"TRACK"`: the internal state tracks `utrack`
- any other value, typically `"AUTO"`: normal closed-loop control

Behavior:

- If `ki == 0`, the controller behaves as a position-form controller without integral action, which allows P or PD operation.
- If `ki ~= 0`, the controller uses an incremental form with anti-windup back-calculation.
- Saturation and rate limits are always applied to the final output.

#### `xf2 = filter(obj, y)`

Applies the built-in second-order measurement filter and returns the filtered signal. The examples use this method when noise attenuation is desired.

#### `resetIntegratorLikeStates(obj)`

Soft reset for derivative and increment-related memory while preserving output continuity.

## Control Features Demonstrated

### 1. MAN/AUTO Transfer

`Example1_paper.m` shows how the controller can switch from manual mode to automatic mode without rebuilding the controller object.

### 2. P and PD Operation

`Example2_paper.m` demonstrates that setting `ki = 0` turns the implementation into a controller without integral action. The same example also contains optional code to activate output rate limiting.

### 3. Setpoint Weighting

`Example3_paper.m` compares three controllers with `b = 1`, `b = 0.5`, and `b = 0`. This is used to smooth control effort during reference changes while preserving disturbance rejection behavior.

### 4. Feedforward and Anti-Windup

`Example4_paper.m` compares four cases:

- no saturation
- external saturation without anti-windup
- internal clamping-like behavior using hard limits and very small `Tt`
- internal back-calculation tracking

This example also injects a measurable disturbance and compensates it using a discrete feedforward path.

### 5. Measurement Filtering

`Example5_paper.m` adds white noise to the process output and compares:

- no filtering
- filtering with `Tf = 0.1*Ti`
- filtering with `Tf = 0.01*Ti`

### 6. Gain Scheduling

`Example6_paper.m` uses a nonlinear tank-level process and compares two gain-scheduling strategies:

- multiple controllers running in parallel, with one active and the others in tracking mode
- a single controller whose gains are updated online according to the operating region

### 7. Selector / Override Control

`Example7_paper.m` shows how to use the tracking mode when two controllers compete through a selector. The active controller drives the plant, while the inactive one tracks the selected output to avoid windup and switching bumps.

## Shared Support Scripts

### `process_and_simulation_data.m`

This script works as a centralized parameter loader. Depending on `exampleID`, it defines:

- process model parameters
- initial conditions
- simulation time and sampling time
- preallocated arrays
- noise or disturbance signals
- special data for each example

### `show_graphical_results.m`

This script creates the figures associated with each example. It trims the delay buffer region, assembles the time axis, and generates the final comparative plots.

### `tightfig.m`

Utility function used to reduce excess margins in the generated figures.

## How to Run the Project

Open MATLAB, change the current folder to the repository root, and run any example script directly, for example:

```matlab
Example_basic
```

or:

```matlab
Example4_paper
```

No external toolbox is explicitly required by the code in this repository beyond standard MATLAB language features for classes, plotting, and `.mat` file loading.

## Minimal Usage Example

The essential workflow is:

```matlab
pid = PID();

pid.Dt = 0.01;
pid.kp = 1.0;
pid.ki = 0.2;
pid.kd = 0.0;
pid.b = 1;
pid.c = 0;
pid.Tt = 1.0;
pid.minlim = -Inf;
pid.maxlim = Inf;
pid.dumin = -Inf;
pid.dumax = Inf;
pid.Tf = 0.01;

pid.init(0, 0, 0, 0);

yf = pid.filter(y);
u = pid.control(r, yf, 0, 0, 0, "AUTO");
```

## Notes and Practical Details

- The examples are scripts, not functions. They operate on variables created in the base workspace.
- The `@PID` folder must remain named exactly as `@PID` so MATLAB recognizes the class.
- Example 5 uses `semilla.mat` to keep the random noise repeatable.
- Example 7 uses `v.mat` as a predefined disturbance profile.
- Some files with extension `.asv` are MATLAB autosave backups and are not required for normal use.
- `Example7.zip` appears to be an auxiliary archive and is not referenced by the main scripts.

## Recommended Starting Point

If you are new to the repository, run the files in this order:

1. `Example_basic.m`
2. `Example1_paper.m`
3. `Example3_paper.m`
4. `Example4_paper.m`
5. `Example6_paper.m`
6. `Example7_paper.m`

This order moves from a simple closed-loop simulation to the more advanced industrial-control use cases.

