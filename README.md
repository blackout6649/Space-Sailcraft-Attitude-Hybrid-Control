# Space-Sailcraft-Attitude-Hybrid-Control

Main entry points:

- `solar_sail_attitude_control_Comparison()` runs the default case and saves the result in `solarSailResult` when called without an output.
- `solar_sail_hybrid_pid_simulate(parameters, controller, scenario)` runs one hybrid PID scenario and returns the result struct.
- `solar_sail_hybrid_pid_defaults()` returns the default parameter, controller, and scenario structs.
- `solar_sail_hybrid_pid_plot_results(result, scenario)` plots attitude, rate, actuator, and torque histories from a simulation result.

Use the wrapper when you just want to press Run. Use the core function when you want to change parameters, controller settings, or scenario settings.
Set `scenario.savePlots = true` and `scenario.plotFolder = 'plots'` to save PNGs into a dedicated plots folder.

Example:

```matlab
solar_sail_attitude_control_Comparison

defaults = solar_sail_hybrid_pid_defaults();
result = solar_sail_hybrid_pid_simulate(defaults.parameters, defaults.controller, defaults.scenario);
```

The result includes state histories, Euler-angle histories, actuator histories, commanded quantities, torque decomposition histories, prepared parameters, and summary metrics.