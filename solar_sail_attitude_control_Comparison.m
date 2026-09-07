function result = solar_sail_attitude_control_Comparison(Euler0,AngularRate0)
%This function is used to run the simulation for given specific initial
%conditions.
    defaults = solar_sail_hybrid_pid_defaults(Euler0, AngularRate0);%Set parameters
    result = solar_sail_hybrid_pid_simulate(defaults.parameters, defaults.controller, defaults.scenario);%Run simulatuion
    solar_sail_hybrid_pid_plot_results(result, defaults.scenario);%Plot resutls graphs
    if nargout == 0
	    assignin('base', 'solarSailResult', result);
    end
end