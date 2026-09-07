function plotHandles = solar_sail_hybrid_pid_plot_results(result, scenario)
%SOLAR_SAIL_HYBRID_PID_PLOT_RESULTS  Five figures:
%   1) attitude error and angular-rate error   (2 subplots)
%   2) all actuator histories                  (4 subplots)
%   3) commanded vs actual moments
%   4) moment breakdown, vane / RCD / disturbance (3 subplots)
%   5) sun angle error

if nargin < 2
    scenario = struct();
end
if ~isfield(scenario, 'savePlots')
    scenario.savePlots = false;
end
if ~isfield(scenario, 'plotFolder') || isempty(scenario.plotFolder)
    scenario.plotFolder = 'plots';
end

% --- Prepare plotted histories --------------------------------------
timeHours = result.time(:) / 3600;
attitudeErrorDeg = rad2deg(result.attitudeError321);
angularRateDegPerSec = rad2deg(result.state(:, 5:7));
vaneCommandDeg = rad2deg(result.command.vane);
vaneActualDeg = rad2deg(result.vaneDeflection);
rcdCommand = result.command.rcd;
rcdActual = result.rcdReflectivity;
sunAngleErrorDeg = rad2deg(result.sunAngle(:) - model_sun_angle(result));
commandTorque = result.command.torque;
delayedTorque = result.torque.afterActuatorDelay;
vaneCommandTorque = result.torque.vanesCommanded;
rcdCommandTorque = result.torque.rcdCommanded;
disturbanceTorque = result.torque.disturbance;

if scenario.savePlots && ~exist(scenario.plotFolder, 'dir')
    mkdir(scenario.plotFolder);
end

blue = [0 0.45 0.74];
orange = [0.85 0.33 0.10];
green = [0.20 0.62 0.20];
purple = [0.49 0.18 0.56];
limitColor = [0.15 0.15 0.15];

plotHandles = struct();

% =====================================================================
% Figure 1 : attitude error and angular rate error
% =====================================================================
plotHandles.attitude = figure('Visible', 'on', 'Color', 'w');

subplot(2, 1, 1); hold on; grid on; box on;
plot(timeHours, attitudeErrorDeg(:, 3), 'LineWidth', 1.6);
plot(timeHours, attitudeErrorDeg(:, 2), 'LineWidth', 1.6);
plot(timeHours, attitudeErrorDeg(:, 1), 'LineWidth', 1.6);
xlabel('time [h]');
ylabel('attitude error [deg]');
title('Yaw, pitch, roll attitude error');
legend({'yaw error', 'pitch error', 'roll error'}, 'Location', 'best');

subplot(2, 1, 2); hold on; grid on; box on;
plot(timeHours, angularRateDegPerSec(:, 3), 'LineWidth', 1.6);
plot(timeHours, angularRateDegPerSec(:, 2), 'LineWidth', 1.6);
plot(timeHours, angularRateDegPerSec(:, 1), 'LineWidth', 1.6);
xlabel('time [h]');
ylabel('angular velocity [deg/s]');
title('Angular velocity error');
legend({'yaw rate', 'pitch rate', 'roll rate'}, 'Location', 'best');

% =====================================================================
% Figure 2 : all actuator histories, commanded vs actual
% =====================================================================
plotHandles.actuators = figure('Visible', 'on', 'Color', 'w');

rollYawLimitDeg = model_limit_deg(result, 'rollYaw');
pitchLimitDeg = model_limit_deg(result, 'pitch');
rcdLimit = model_limit_rcd(result);

% ---- roll vanes -----------------------------------------------------
subplot(2, 2, 1); hold on; grid on; box on;
h1 = plot(timeHours, vaneCommandDeg(:, 1), '--', 'Color', blue, 'LineWidth', 1.4);
h2 = plot(timeHours, vaneActualDeg(:, 1), '-', 'Color', blue, 'LineWidth', 1.4);
h3 = plot(timeHours, vaneCommandDeg(:, 4), '--', 'Color', orange, 'LineWidth', 1.4);
h4 = plot(timeHours, vaneActualDeg(:, 4), '-', 'Color', orange, 'LineWidth', 1.4);
hs = plot(timeHours, rollYawLimitDeg, ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -rollYawLimitDeg, ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('deflection [deg]');
title('Roll vanes: commanded vs actual');
legend([h1 h2 h3 h4 hs], ...
    {'\delta_1 cmd', '\delta_1 actual', '\delta_4 cmd', '\delta_4 actual', 'Saturation'}, ...
    'Location', 'bestoutside');

% ---- pitch vanes ----------------------------------------------------
subplot(2, 2, 2); hold on; grid on; box on;
h1 = plot(timeHours, vaneCommandDeg(:, 2), '--', 'Color', green, 'LineWidth', 1.4);
h2 = plot(timeHours, vaneActualDeg(:, 2), '-', 'Color', green, 'LineWidth', 1.4);
h3 = plot(timeHours, vaneCommandDeg(:, 3), '--', 'Color', purple, 'LineWidth', 1.4);
h4 = plot(timeHours, vaneActualDeg(:, 3), '-', 'Color', purple, 'LineWidth', 1.4);
hs = plot(timeHours, pitchLimitDeg, ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -pitchLimitDeg, ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('deflection [deg]');
title('Pitch vanes: commanded vs actual');
legend([h1 h2 h3 h4 hs], ...
    {'\delta_2 cmd', '\delta_2 actual', '\delta_3 cmd', '\delta_3 actual', 'Saturation'}, ...
    'Location', 'bestoutside');

% ---- RCD quadrants TR and TL ---------------------------------------
subplot(2, 2, 3); hold on; grid on; box on;
h1 = plot(timeHours, rcdCommand(:, 1), '--', 'Color', blue, 'LineWidth', 1.3);
h2 = plot(timeHours, rcdActual(:, 1), '-', 'Color', blue, 'LineWidth', 1.3);
h3 = plot(timeHours, rcdCommand(:, 2), '--', 'Color', orange, 'LineWidth', 1.3);
h4 = plot(timeHours, rcdActual(:, 2), '-', 'Color', orange, 'LineWidth', 1.3);
hs = plot(timeHours, rcdLimit, ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -rcdLimit, ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('\Delta\rho');
title('RCD quadrant 1 and 2: commanded vs actual');
legend([h1 h2 h3 h4 hs], ...
    {'TR cmd', 'TR actual', 'TL cmd', 'TL actual', 'Saturation'}, 'Location', 'bestoutside');

% ---- RCD quadrants BL and BR ---------------------------------------
subplot(2, 2, 4); hold on; grid on; box on;
h1 = plot(timeHours, rcdCommand(:, 3), '--', 'Color', green, 'LineWidth', 1.3);
h2 = plot(timeHours, rcdActual(:, 3), '-', 'Color', green, 'LineWidth', 1.3);
h3 = plot(timeHours, rcdCommand(:, 4), '--', 'Color', purple, 'LineWidth', 1.3);
h4 = plot(timeHours, rcdActual(:, 4), '-', 'Color', purple, 'LineWidth', 1.3);
hs = plot(timeHours, rcdLimit, ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -rcdLimit, ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('\Delta\rho');
title('RCD quadrant 3 and 4: commanded vs actual');
legend([h1 h2 h3 h4 hs], ...
    {'BL cmd', 'BL actual', 'BR cmd', 'BR actual', 'Saturation'}, 'Location', 'bestoutside');

% =====================================================================
% Figure 3 : commanded vs post-actuator-delay moments
% =====================================================================
plotHandles.torqueCommandedVsTotal = figure('Visible', 'on', 'Color', 'w');
hold on; grid on; box on;
plot(timeHours, commandTorque(:, 1), '-', 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, delayedTorque(:, 1), '--', 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, commandTorque(:, 2), '-', 'Color', green, 'LineWidth', 1.4);
plot(timeHours, delayedTorque(:, 2), '--', 'Color', green, 'LineWidth', 1.4);
plot(timeHours, commandTorque(:, 3), '-', 'Color', orange, 'LineWidth', 1.4);
plot(timeHours, delayedTorque(:, 3), '--', 'Color', orange, 'LineWidth', 1.4);
xlabel('time [h]');
ylabel('torque [N m]');
title('Commanded vs after-actuator-delay moments');
legend({'T_x commanded', 'T_x after actuator delay', ...
    'T_y commanded', 'T_y after actuator delay', ...
    'T_z commanded', 'T_z after actuator delay'}, ...
    'Location', 'bestoutside');

% =====================================================================
% Figure 4 : commanded moment breakdown -- vane, RCD and disturbance
% =====================================================================
plotHandles.torqueBreakdown = figure('Visible', 'on', 'Color', 'w');

subplot(3, 1, 1); hold on; grid on; box on;
plot(timeHours, commandTorque(:, 1), 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneCommandTorque(:, 1), '--', 'Color', orange, 'LineWidth', 1.2);
plot(timeHours, rcdCommandTorque(:, 1), ':', 'Color', green, 'LineWidth', 1.2);
plot(timeHours, disturbanceTorque(:, 1), '--', 'Color', [0 0 0], 'LineWidth', 1.2);
ylabel('T_x [N m]');
title('Commanded roll-axis moments');
legend({'commanded total', 'commanded vane', 'commanded RCD', 'disturbance'}, 'Location', 'bestoutside');

subplot(3, 1, 2); hold on; grid on; box on;
plot(timeHours, commandTorque(:, 2), 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneCommandTorque(:, 2), '--', 'Color', orange, 'LineWidth', 1.2);
plot(timeHours, rcdCommandTorque(:, 2), ':', 'Color', green, 'LineWidth', 1.2);
plot(timeHours, disturbanceTorque(:, 2), '--', 'Color', [0 0 0], 'LineWidth', 1.2);
ylabel('T_y [N m]');
title('Commanded pitch-axis moments');
legend({'commanded total', 'commanded vane', 'commanded RCD', 'disturbance'}, 'Location', 'bestoutside');

subplot(3, 1, 3); hold on; grid on; box on;
plot(timeHours, commandTorque(:, 3), 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneCommandTorque(:, 3), '--', 'Color', orange, 'LineWidth', 1.2);
plot(timeHours, rcdCommandTorque(:, 3), ':', 'Color', green, 'LineWidth', 1.2);
plot(timeHours, disturbanceTorque(:, 3), '--', 'Color', [0 0 0], 'LineWidth', 1.2);
xlabel('time [h]');
ylabel('T_z [N m]');
title('Commanded yaw-axis moments');
legend({'commanded total', 'commanded vane', 'commanded RCD', 'disturbance'}, 'Location', 'bestoutside');

% =====================================================================
% Figure 5 : sun angle error
% =====================================================================
plotHandles.sunAngle = figure('Visible', 'on', 'Color', 'w');
hold on; grid on; box on;
plot(timeHours, sunAngleErrorDeg, 'LineWidth', 1.6);
xlabel('time [h]');
ylabel('sun angle error [deg]');
title('Sun angle error');

if scenario.savePlots
    saveas(plotHandles.attitude, fullfile(scenario.plotFolder, 'attitude_and_rate_error.png'));
    saveas(plotHandles.actuators, fullfile(scenario.plotFolder, 'actuators.png'));
    saveas(plotHandles.torqueCommandedVsTotal, fullfile(scenario.plotFolder, 'required_moments_commanded_vs_total.png'));
    saveas(plotHandles.torqueBreakdown, fullfile(scenario.plotFolder, 'required_moments_breakdown.png'));
    saveas(plotHandles.sunAngle, fullfile(scenario.plotFolder, 'sun_angle.png'));
end
end

function alphaCmd = model_sun_angle(result)
alphaCmd = result.parameters.environment.alphaCmd;
end

function limitDeg = model_limit_deg(result, kind)
switch kind
    case 'rollYaw'
        limitDeg = rad2deg(result.parameters.vanes.deflectionLimit) * ones(size(result.time(:)));
    case 'pitch'
        if isfield(result.parameters.vanes, 'pitchDeflectionLimit') && ~isempty(result.parameters.vanes.pitchDeflectionLimit)
            limitDeg = rad2deg(result.parameters.vanes.pitchDeflectionLimit) * ones(size(result.time(:)));
        else
            limitDeg = rad2deg(result.parameters.vanes.deflectionLimit) * ones(size(result.time(:)));
        end
    otherwise
        error('Unknown limit kind.');
end
end

function limitRcd = model_limit_rcd(result)
limitRcd = result.parameters.rcd.reflectivityLimit * ones(size(result.time(:)));
end
