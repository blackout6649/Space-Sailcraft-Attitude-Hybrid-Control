# Space-Sailcraft-Attitude-Hybrid-Control

A MATLAB simulation framework for solar sailcraft attitude control using hybrid vane/RCD (Reflectivity Control Device) PID controllers.

## Main Entry Points

- **`solar_sail_attitude_control_Comparison(Euler0, AngularRate0)`** — Wrapper that runs a complete simulation with the given initial conditions and plots results. Returns result to `solarSailResult` when called without output argument.
  
- **`solar_sail_hybrid_pid_simulate(parameters, controller, scenario)`** — Core simulation function. Runs one hybrid PID scenario and returns the result struct. If called with no inputs, uses defaults.

- **`solar_sail_hybrid_pid_defaults(Euler0, AngularRate0)`** — Returns default parameter, controller, and scenario structs. Can optionally accept initial Euler angles and angular rates.

- **`solar_sail_hybrid_pid_plot_results(result, scenario)`** — Plots five figures showing attitude/rate errors, actuator histories, commanded vs. actual moments, moment breakdown, and sun angle error.

- **`solar_sail_attitude_control_Main.m`** — Batch runner that executes multiple simulation cases with different initial conditions.

## Usage Examples

**Quick run with defaults:**
```matlab
solar_sail_attitude_control_Comparison()
```

**Run with custom initial conditions:**
```matlab
Euler0 = [55; 55; 55] * pi/180;        % Radians
AngularRate0 = [0.02; 0.02; 0.02];     % rad/s
solar_sail_attitude_control_Comparison(Euler0, AngularRate0)
```

**Custom simulation with modified parameters:**
```matlab
defaults = solar_sail_hybrid_pid_defaults(Euler0, AngularRate0);
defaults.controller.wn = [1.5e-3 1.5e-3 1.5e-3];  % Adjust natural frequencies
result = solar_sail_hybrid_pid_simulate(defaults.parameters, defaults.controller, defaults.scenario);
solar_sail_hybrid_pid_plot_results(result, defaults.scenario);
```

**Save plots to folder:**
```matlab
defaults = solar_sail_hybrid_pid_defaults();
defaults.scenario.savePlots = true;
defaults.scenario.plotFolder = 'plots';
result = solar_sail_hybrid_pid_simulate(defaults.parameters, defaults.controller, defaults.scenario);
solar_sail_hybrid_pid_plot_results(result, defaults.scenario);
```

## Output

Simulation results include:
- State histories (position, velocity, attitude, rates)
- Euler angle histories (321 sequence)
- Actuator command and actual deflection histories (vanes and RCD)
- Torque decomposition (vanes, RCD, disturbance contributions)
- Commanded vs. actual control moments
- Summary metrics and performance data