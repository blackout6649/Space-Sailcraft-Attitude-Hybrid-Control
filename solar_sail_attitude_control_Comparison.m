function result = solar_sail_attitude_control_Comparison()
defaults = solar_sail_hybrid_pid_defaults();
result = solar_sail_hybrid_pid_simulate(defaults.parameters, defaults.controller, defaults.scenario);
solar_sail_hybrid_pid_plot_results(result, defaults.scenario);

if nargout == 0
	assignin('base', 'solarSailResult', result);
end
end