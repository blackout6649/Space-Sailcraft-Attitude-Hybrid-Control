clc;
clearvars;
close all;

% Batch Simulation Runner: Execute multiple attitude control scenarios
% Compares controller performance across different initial conditions

% --- Define Test Cases -------------------------------------------
% Initial conditions: Euler angles in degrees, angular rates in rad/s
simulationCases = [
	struct('name', 'Zero attitude and rate', ...
		'eulerAnglesDeg', [0; 0; 0], ...
		'angularRateRadPerSec', [0; 0; 0]), ...
	struct('name', '55 degree attitude offset', ...
		'eulerAnglesDeg', [55; 55; 55], ...
		'angularRateRadPerSec', [0; 0; 0]), ...
	struct('name', 'Initial angular rate', ...
		'eulerAnglesDeg', [0; 0; 0], ...
		'angularRateRadPerSec', [0.02; 0.02; 0.02])
];

% --- Execute Each Case -----------------------------------------------
for caseIndex = 1:numel(simulationCases)
	simulationCase = simulationCases(caseIndex);
	fprintf('Running case %d: %s\n', caseIndex, simulationCase.name);

	initialEulerRad = deg2rad(simulationCase.eulerAnglesDeg);
	initialAngularRateRadPerSec = simulationCase.angularRateRadPerSec;

	solar_sail_attitude_control_Comparison( ...
		initialEulerRad, initialAngularRateRadPerSec);
end