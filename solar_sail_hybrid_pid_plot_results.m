function plotHandles = solar_sail_hybrid_pid_plot_results(result, scenario)
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
sunAngleErrorDeg = rad2deg(result.sunAngle(:) - model_sun_angle(result));
vaneCommandDeg = rad2deg(result.command.vane);
vaneActualDeg = rad2deg(result.vaneDeflection);
rcdCommand = result.command.rcd;
rcdActual = result.rcdReflectivity;
commandTorque = result.command.torque;
totalTorque = result.torque.total;
vaneTorque = result.torque.vanes;
rcdTorque = result.torque.rcd;
disturbanceTorque = result.torque.disturbance;

if scenario.savePlots
    if ~exist(scenario.plotFolder, 'dir')
        mkdir(scenario.plotFolder);
    end
end

plotHandles = struct();

% --- Attitude error and angular rate --------------------------------
plotHandles.attitude = figure('Visible', 'on', 'Color', 'w');
hold on; grid on; box on;
plot(timeHours, attitudeErrorDeg(:, 3), 'LineWidth', 1.6);
plot(timeHours, attitudeErrorDeg(:, 2), 'LineWidth', 1.6);
plot(timeHours, attitudeErrorDeg(:, 1), 'LineWidth', 1.6);
xlabel('time [h]');
ylabel('attitude error [deg]');
title('Yaw, pitch, roll attitude error');
legend({'yaw error', 'pitch error', 'roll error'}, 'Location', 'best');

plotHandles.angularVelocity = figure('Visible', 'on', 'Color', 'w');
hold on; grid on; box on;
plot(timeHours, angularRateDegPerSec(:, 3), 'LineWidth', 1.6);
plot(timeHours, angularRateDegPerSec(:, 2), 'LineWidth', 1.6);
plot(timeHours, angularRateDegPerSec(:, 1), 'LineWidth', 1.6);
xlabel('time [h]');
ylabel('angular velocity [deg/s]');
title('Angular velocity error');
legend({'yaw rate', 'pitch rate', 'roll rate'}, 'Location', 'best');

% --- Actuator command vs actual histories ---------------------------
plotHandles.vanes = figure('Visible', 'on', 'Color', 'w');
subplot(2,1,1); hold on; grid on; box on;
blue = [0 0.45 0.74];
orange = [0.85 0.33 0.10];
green = [0.20 0.62 0.20];
purple = [0.49 0.18 0.56];
limitColor = [0.15 0.15 0.15];
plot(timeHours, vaneCommandDeg(:, 1), '--', 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneActualDeg(:, 1), '-', 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneCommandDeg(:, 4), '--', 'Color', orange, 'LineWidth', 1.4);
plot(timeHours, vaneActualDeg(:, 4), '-', 'Color', orange, 'LineWidth', 1.4);
plot(timeHours, model_limit_deg(result, 'rollYaw'), ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -model_limit_deg(result, 'rollYaw'), ':', 'Color', limitColor, 'LineWidth', 1.1);
ylabel('deflection [deg]');
title('Roll/yaw vanes: commanded vs actual');
legend({'\delta_1 cmd', '\delta_1 actual', '\delta_4 cmd', '\delta_4 actual'}, 'Location', 'best');
subplot(2,1,2); hold on; grid on; box on;
plot(timeHours, vaneCommandDeg(:, 2), '--', 'Color', green, 'LineWidth', 1.4);
plot(timeHours, vaneActualDeg(:, 2), '-', 'Color', green, 'LineWidth', 1.4);
plot(timeHours, vaneCommandDeg(:, 3), '--', 'Color', purple, 'LineWidth', 1.4);
plot(timeHours, vaneActualDeg(:, 3), '-', 'Color', purple, 'LineWidth', 1.4);
plot(timeHours, model_limit_deg(result, 'pitch'), ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -model_limit_deg(result, 'pitch'), ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('deflection [deg]');
title('Pitch vanes: commanded vs actual');
legend({'\delta_2 cmd', '\delta_2 actual', '\delta_3 cmd', '\delta_3 actual'}, 'Location', 'best');

plotHandles.rcd = figure('Visible', 'on', 'Color', 'w');
subplot(2,1,1); hold on; grid on; box on;
plot(timeHours, rcdCommand(:, 1), '--', 'Color', blue, 'LineWidth', 1.3);
plot(timeHours, rcdActual(:, 1), '-', 'Color', blue, 'LineWidth', 1.3);
plot(timeHours, rcdCommand(:, 2), '--', 'Color', orange, 'LineWidth', 1.3);
plot(timeHours, rcdActual(:, 2), '-', 'Color', orange, 'LineWidth', 1.3);
plot(timeHours, model_limit_rcd(result), ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -model_limit_rcd(result), ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('\Delta\rho');
title('RCD quadrant 1 and 2: commanded vs actual');
legend({'TR cmd', 'TR actual', 'TL cmd', 'TL actual'}, 'Location', 'best');
subplot(2,1,2); hold on; grid on; box on;
plot(timeHours, rcdCommand(:, 3), '--', 'Color', green, 'LineWidth', 1.3);
plot(timeHours, rcdActual(:, 3), '-', 'Color', green, 'LineWidth', 1.3);
plot(timeHours, rcdCommand(:, 4), '--', 'Color', purple, 'LineWidth', 1.3);
plot(timeHours, rcdActual(:, 4), '-', 'Color', purple, 'LineWidth', 1.3);
plot(timeHours, model_limit_rcd(result), ':', 'Color', limitColor, 'LineWidth', 1.1);
plot(timeHours, -model_limit_rcd(result), ':', 'Color', limitColor, 'LineWidth', 1.1);
xlabel('time [h]');
ylabel('\Delta\rho');
title('RCD quadrant 3 and 4: commanded vs actual');
legend({'BL cmd', 'BL actual', 'BR cmd', 'BR actual'}, 'Location', 'best');

% --- Required moments and additional diagnostic ---------------------
plotHandles.torqueCommandedVsTotal = figure('Visible', 'on', 'Color', 'w');
hold on; grid on; box on;
plot(timeHours, commandTorque(:, 1),'-', 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, totalTorque(:, 1),'--', 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, commandTorque(:, 2),'-', 'Color', green, 'LineWidth', 1.4);
plot(timeHours, totalTorque(:, 2),'--', 'Color', green, 'LineWidth', 1.4);
plot(timeHours, commandTorque(:, 3),'-', 'Color', orange, 'LineWidth', 1.4);
plot(timeHours, totalTorque(:, 3),'--','Color', orange, 'LineWidth', 1.4);
xlabel('time [h]');
ylabel('torque [N m]');
title('Commanded vs actual moments');
legend({'T_x cmd', 'T_x actual', 'T_y cmd', 'T_y actual', 'T_z cmd', 'T_z actual'}, 'Location', 'best');

plotHandles.torqueBreakdown = figure('Visible', 'on', 'Color', 'w');
subplot(3,1,1); hold on; grid on; box on;
plot(timeHours, totalTorque(:, 1), 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneTorque(:, 1), '--', 'Color', orange, 'LineWidth', 1.2);
plot(timeHours, rcdTorque(:, 1), ':', 'Color', green, 'LineWidth', 1.2);
plot(timeHours, disturbanceTorque(:, 1), '-.', 'Color', purple, 'LineWidth', 1.2);
ylabel('T_x [N m]');
title('Actual vs vane vs RCD moments');
legend({'actual', 'vane', 'RCD', 'disturbance'}, 'Location', 'best');
subplot(3,1,2); hold on; grid on; box on;
plot(timeHours, totalTorque(:, 2), 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneTorque(:, 2), '--', 'Color', orange, 'LineWidth', 1.2);
plot(timeHours, rcdTorque(:, 2), ':', 'Color', green, 'LineWidth', 1.2);
plot(timeHours, disturbanceTorque(:, 2), '-.', 'Color', purple, 'LineWidth', 1.2);
ylabel('T_y [N m]');
title('Pitch-axis moments');
legend({'actual', 'vane', 'RCD', 'disturbance'}, 'Location', 'best');
subplot(3,1,3); hold on; grid on; box on;
plot(timeHours, totalTorque(:, 3), 'Color', blue, 'LineWidth', 1.4);
plot(timeHours, vaneTorque(:, 3), '--', 'Color', orange, 'LineWidth', 1.2);
plot(timeHours, rcdTorque(:, 3), ':', 'Color', green, 'LineWidth', 1.2);
plot(timeHours, disturbanceTorque(:, 3), '-.', 'Color', purple, 'LineWidth', 1.2);
xlabel('time [h]');
ylabel('T_z [N m]');
title('Yaw-axis moments');
legend({'actual', 'vane', 'RCD', 'disturbance'}, 'Location', 'best');

plotHandles.sunAngle = figure('Visible', 'on', 'Color', 'w');
hold on; grid on; box on;
plot(timeHours, sunAngleErrorDeg, 'LineWidth', 1.6);
xlabel('time [h]');
ylabel('sun angle error [deg]');
title('Sun angle error');

if scenario.savePlots
    saveas(plotHandles.attitude, fullfile(scenario.plotFolder, 'attitude_error.png'));
    saveas(plotHandles.angularVelocity, fullfile(scenario.plotFolder, 'angular_velocity.png'));
    saveas(plotHandles.vanes, fullfile(scenario.plotFolder, 'vane_deflection.png'));
    saveas(plotHandles.rcd, fullfile(scenario.plotFolder, 'rcd_modulation.png'));
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