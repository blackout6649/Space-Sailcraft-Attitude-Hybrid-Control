function result = solar_sail_attitude_control_Comparison(Euler0, AngularRate0)
%SOLAR_SAIL_ATTITUDE_CONTROL_COMPARISON Wrapper to run simulation with initial conditions.
%   Convenience function that orchestrates the full simulation pipeline:
%   1) Set up default parameters with given initial conditions
%   2) Run the hybrid PID simulation
%   3) Plot results automatically
%   If called without output, result is assigned to base workspace as solarSailResult.
%
%   Usage: solar_sail_attitude_control_Comparison(Euler0_rad, AngularRate0_rad_per_sec)

    defaults = solar_sail_hybrid_pid_defaults(Euler0, AngularRate0);
    result = solar_sail_hybrid_pid_simulate(defaults.parameters, defaults.controller, defaults.scenario);
    solar_sail_hybrid_pid_plot_results(result, defaults.scenario);
    
    % Return result to base workspace if called without output
    if nargout == 0
        assignin('base', 'solarSailResult', result);
    end
end