# Active-Suspension
3rd year project MATLAB code
# Active Suspension H-Infinity MATLAB Simulation

## Overview

This repository contains a MATLAB script, `test.m`, used to model and test a quarter-car active suspension system. The script compares a passive suspension model with an active suspension model that includes hydraulic actuator dynamics and an H-infinity controller.

The main purpose of the script is to evaluate how the active controller affects:

- suspension travel, `x1 = Zs - Zu`
- tyre deflection, `x3 = Zu - Zr`
- controller current demand, `i(t)`
- actuator force, `Fa`
- passive versus active frequency response
- passive versus active time-domain response

The script is intended to support the active suspension project by generating simulation results, comparison plots, and basic numerical performance metrics.

## Main File

| File | Description |
|---|---|
| `test.m` | Main MATLAB script containing the vehicle model, actuator model, H-infinity controller synthesis, passive comparison model, road input tests, plots, and printed performance values. |

## Requirements

To run the script, the following are required:

- MATLAB
- Control System Toolbox
- Robust Control Toolbox

The script uses MATLAB functions such as:

- `tf`
- `ss`
- `blkdiag`
- `hinfsyn`
- `lft`
- `lsim`
- `bodemag`

A recent MATLAB version is recommended because the script uses robust control synthesis and state-space modelling functions.

## How to Run

1. Open MATLAB.
2. Place `test.m` in the current MATLAB working folder.
3. Make sure the Control System Toolbox and Robust Control Toolbox are installed.
4. Run the script by typing:

```matlab
test
```

or by opening `test.m` and pressing **Run**.

## What the Script Does

The script performs the following main steps:

1. Defines the vehicle and actuator parameters.
2. Builds a five-state active suspension model.
3. Builds a four-state passive suspension model for comparison.
4. Defines H-infinity weighting functions for:
   - tyre deflection
   - suspension travel
   - controller current demand
5. Constructs the weighted generalised plant.
6. Synthesises an H-infinity controller using `hinfsyn`.
7. Forms the closed-loop active system.
8. Compares passive and active responses using Bode plots.
9. Runs time-domain road disturbance tests.
10. Prints RMS, peak, controller current, actuator force, and settling-time values.

## Road Tests Included

The script includes three road input cases:

1. **Continuous sine undulation**  
   A smooth sinusoidal road input used to test repeated low-frequency road excitation.

2. **Kerb half-sine event**  
   A single half-sine bump used to represent a kerb-type disturbance.

3. **Mixed undulation and half-sine kerb event**  
   A combined test with continuous road undulation and a kerb event.

## Outputs

When the script is run, MATLAB produces:

- H-infinity gamma value
- controller order
- Bode plots for passive and active responses
- weighting function Bode plot
- road profile plots
- suspension travel plots
- tyre deflection plots
- controller current demand plots
- actuator force plots
- RMS and peak response values printed in the command window
- settling-time estimates for the half-sine kerb test

## Notes on Results

The active controller is assessed against the passive model using the same road disturbance input. This allows the change in response to be attributed to the controller rather than to a change in road excitation.

The main comparison variables are:

```matlab
x1 = Zs - Zu      % suspension travel
x3 = Zu - Zr      % tyre deflection
u  = i(t)         % controller current
Fa                % actuator force
```
