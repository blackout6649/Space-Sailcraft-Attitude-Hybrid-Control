function result = solar_sail_hybrid_pid_simulate(parameters, controller, scenario)
%SOLAR_SAIL_HYBRID_PID_SIMULATE Run one hybrid vane/RCD PID simulation.

% --- Defaults ---------------------------------------------------------
if nargin == 0
    defaults = solar_sail_hybrid_pid_defaults();
    parameters = defaults.parameters;
    controller = defaults.controller;
    scenario = defaults.scenario;
elseif nargin < 3
    error('Pass parameters, controller, and scenario, or call with no inputs.');
end

% Consolidate all inputs into a single 'model' struct for cleaner code flow
model.initial = parameters.initial;
model.geometry = parameters.geometry;
model.environment = parameters.environment;
model.vanes = parameters.vanes;
model.rcd = parameters.rcd;
model.disturbance = parameters.disturbance;
model.controller = controller;
model.scenario = scenario;

% --- Input Validation and Reshaping -----------------------------------
% Ensure all vector inputs are column vectors
model.initial.absoluteEuler321 = reshape(model.initial.absoluteEuler321, 3, 1);
model.initial.angularRate = reshape(model.initial.angularRate, 3, 1);
model.initial.integralState = reshape(model.initial.integralState, 3, 1);
model.initial.vaneDeflection = reshape(model.initial.vaneDeflection, 4, 1);
model.initial.rcdReflectivity = reshape(model.initial.rcdReflectivity, 4, 1);
model.controller.wn = reshape(model.controller.wn, 1, 3);
model.controller.zeta = reshape(model.controller.zeta, 1, 3);
model.controller.wi = reshape(model.controller.wi, 1, 3);
if ~isfield(model.vanes, 'pitchDeflectionLimit') || isempty(model.vanes.pitchDeflectionLimit)
    model.vanes.pitchDeflectionLimit = model.vanes.deflectionLimit;
end
model.vanes.deflectionLimitVector = [model.vanes.deflectionLimit; model.vanes.pitchDeflectionLimit; model.vanes.pitchDeflectionLimit; model.vanes.deflectionLimit];
if isfield(model.controller, 'useVaneYaw')
    error('controller.useVaneYaw is no longer supported. Hybrid yaw control is RCD-only.');
end

if abs(model.geometry.sailArea - model.geometry.sailSide ^ 2) > 1e-9
    error('geometry.sailArea must equal geometry.sailSide^2.');
end


% --- Derived quantities -----------------------------------------------

model.derived.inertia = model.geometry.inertia;
model.derived.inertiaInverse = inv(model.geometry.inertia);
model.derived.trimAlpha = atan(1 / sqrt(2));
model.derived.vaneTipDistance = model.geometry.sailSide / sqrt(2);
model.derived.vaneMomentArm = model.derived.vaneTipDistance;                    % L, Sec. 2.4.1
model.derived.vaneForce = model.environment.eta * model.environment.Psrp * model.geometry.vaneArea;  % Fc = eta P Ac, Sec. 2.4.1
model.derived.solarForce = model.environment.eta * model.environment.Psrp * model.geometry.sailArea;

principalInertia = diag(model.derived.inertia).';
model.derived.Kd = diag(principalInertia .* (2 * model.controller.zeta .* model.controller.wn + model.controller.wi));
model.derived.Kp = diag(2 * principalInertia .* (model.controller.wn .^ 2 + 2 * model.controller.zeta .* model.controller.wn .* model.controller.wi));
model.derived.Ki = diag(2 * principalInertia .* model.controller.wn .^ 2 .* model.controller.wi);
model.derived.integralLimit = model.controller.Tintmx ./ max(diag(model.derived.Ki).', 1e-30);

% Desired attitude: pitch to align with sun-line (Euler angle 2)
targetDcm = C2(-model.environment.alphaCmd);
model.derived.qd = dcm2quat(targetDcm);

% The user provides the initial body attitude in the sun-line frame.
initialBodyDcm = C1(model.initial.absoluteEuler321(1)) * C2(model.initial.absoluteEuler321(2)) * C3(model.initial.absoluteEuler321(3));
qT0 = dcm2quat(initialBodyDcm);
model.initial.derivedSunAngle = acos(max(-1, min(1, initialBodyDcm(1, 1))));
model.initial.derivedSunAngleError = model.initial.derivedSunAngle - model.environment.alphaCmd;

% State vector: [qT (4), omegaT (3), z (3), deltaT (4), rhoT (4)] = 18 states
% qT: quaternion (body relative to sun-line frame)
% omegaT: angular velocity
% z: integral state for PID controller
% deltaT: vane deflection angles [roll1, pitch2, pitch3, roll4]
% rhoT: RCD reflectivity modulation [quadrant1-4]
x0 = [qT0; model.initial.angularRate; model.initial.integralState; model.initial.vaneDeflection; model.initial.rcdReflectivity];
stepCount = round(model.scenario.tEnd / model.scenario.dt);
tGrid = (0:stepCount).' * model.scenario.dt;

% Integrate full closed-loop dynamics using ode45, then resample to uniform time grid
[solverTime, solverState] = ode45(@(time, state) state_derivative(time, state, model), [tGrid(1) tGrid(end)], x0);
stateHistory = interp1(solverTime, solverState, tGrid, 'linear', 'extrap');
stateHistory(:, 1:4) = normalize_quaternion_history(stateHistory(:, 1:4));

% --- Output storage ---------------------------------------------------
result.time = tGrid;
result.state = stateHistory;
result.attitudeError321 = zeros(stepCount + 1, 3);
result.absoluteEuler321 = zeros(stepCount + 1, 3);
result.sunAngle = zeros(stepCount + 1, 1);
result.integralState = zeros(stepCount + 1, 3);
result.vaneDeflection = zeros(stepCount + 1, 4);
result.rcdReflectivity = zeros(stepCount + 1, 4);
result.command.vane = zeros(stepCount + 1, 4);
result.command.rcd = zeros(stepCount + 1, 4);
result.command.torque = zeros(stepCount + 1, 3);
result.command.pitchShare = zeros(stepCount + 1, 1);
result.command.theta = zeros(stepCount + 1, 1);
result.command.delta = zeros(stepCount + 1, 1);
result.torque.vanes = zeros(stepCount + 1, 3);
result.torque.rcd = zeros(stepCount + 1, 3);
result.torque.vanesCommanded = zeros(stepCount + 1, 3);
result.torque.rcdCommanded = zeros(stepCount + 1, 3);
result.torque.disturbance = zeros(stepCount + 1, 3);
result.torque.total = zeros(stepCount + 1, 3);
result.torque.afterActuatorDelay = zeros(stepCount + 1, 3);

% --- Sample outputs on the requested time grid -----------------------
% Extract control, torque, and error signals at each time step for plotting
for stepIndex = 1:(stepCount + 1)
    sample = sample_state(stateHistory(stepIndex, :).', model);
    result.attitudeError321(stepIndex, :) = sample.attitudeError321.';
    result.absoluteEuler321(stepIndex, :) = sample.absoluteEuler321.';
    result.sunAngle(stepIndex) = sample.sunAngle;
    result.integralState(stepIndex, :) = stateHistory(stepIndex, 8:10);
    result.vaneDeflection(stepIndex, :) = stateHistory(stepIndex, 11:14);
    result.rcdReflectivity(stepIndex, :) = stateHistory(stepIndex, 15:18);
    result.command.vane(stepIndex, :) = sample.deltaTcmd.';
    result.command.rcd(stepIndex, :) = sample.deltaRhoTcmd.';
    result.command.torque(stepIndex, :) = sample.commandTorque.';
    result.command.pitchShare(stepIndex) = sample.gammaY;
    result.command.theta(stepIndex) = sample.ThetaCmd;
    result.command.delta(stepIndex) = sample.DeltaCmd;
    result.torque.vanes(stepIndex, :) = sample.Tvanes.';
    result.torque.rcd(stepIndex, :) = sample.Trcd.';
    result.torque.vanesCommanded(stepIndex, :) = sample.TvanesCmd.';
    result.torque.rcdCommanded(stepIndex, :) = sample.TrcdCmd.';
    result.torque.disturbance(stepIndex, :) = sample.Tdist.';
    result.torque.total(stepIndex, :) = sample.Ttotal.';
    result.torque.afterActuatorDelay(stepIndex, :) = sample.Tactuator.';
end

% --- Summary metrics --------------------------------------------------
windowStart = max(1, round((1 - model.scenario.summaryWindowFraction) * numel(tGrid)));
windowIndex = windowStart:numel(tGrid);
result.parameters = model;
result.summary.finalAttitudeError321 = result.attitudeError321(end, :);
result.summary.finalAbsoluteEuler321 = result.absoluteEuler321(end, :);
result.summary.finalSunAngle = result.sunAngle(end);
result.summary.meanAttitudeError321LastWindow = mean(result.attitudeError321(windowIndex, :), 1);
result.summary.maxAbsVaneDeflection = max(max(abs(result.vaneDeflection)));
result.summary.maxAbsRcdReflectivity = max(max(abs(result.rcdReflectivity)));
result.summary.maxAbsCommandTorque = max(max(abs(result.command.torque)));
result.summary.maxAbsTotalTorque = max(max(abs(result.torque.total)));
result.summary.vaneDeflectionSaturated = any(any(abs(result.vaneDeflection) >= (model.vanes.deflectionLimitVector.' - 1e-12)));
result.summary.rollYawVaneSaturated = any(any(abs(result.vaneDeflection(:, [1 4])) >= model.vanes.deflectionLimit - 1e-12));
result.summary.pitchVaneSaturated = any(any(abs(result.vaneDeflection(:, [2 3])) >= model.vanes.pitchDeflectionLimit - 1e-12));
result.summary.rcdReflectivitySaturated = any(any(abs(result.rcdReflectivity) >= model.rcd.reflectivityLimit - 1e-12));
end

% --- Closed-loop dynamics for ode45 ----------------------------------
function derivative = state_derivative(~, state, model)
sample = sample_state(state, model);
qT = state(1:4) / norm(state(1:4));
omegaT = state(5:7);
z = state(8:10);
deltaT = state(11:14);
deltaRhoT = state(15:18);

% Euler rotational equations: dW/dt = J^-1 * (tau - W x J*W)
omegaTdot = model.derived.inertiaInverse * (-cross(omegaT, model.derived.inertia * omegaT) + sample.Ttotal);
qTVec = qT(1:3);
qTScalar = qT(4);
qTVecDot = 0.5 * (qTScalar * eye(3) + skew(qTVec)) * omegaT;
qTScalarDot = -0.5 * (qTVec.' * omegaT);

% First-order actuator dynamics with saturation and rate limits
deltaTdot = sat((sat(sample.deltaTcmd, model.vanes.deflectionLimitVector) - deltaT) / model.vanes.timeConstant, model.vanes.rateLimit);
deltaRhoTdot = (sat(sample.deltaRhoTcmd, model.rcd.reflectivityLimit) - deltaRhoT) / model.rcd.timeConstant;

% Integral error accumulation with anti-windup protection
zdot = sample.qeVec;
for axisIndex = 1:3
    if abs(z(axisIndex)) >= model.derived.integralLimit(axisIndex) && sign(zdot(axisIndex)) == sign(z(axisIndex))
        zdot(axisIndex) = 0;
    end
end

derivative = [qTVecDot; qTScalarDot; omegaTdot; zdot; deltaTdot; deltaRhoTdot];
end

% --- Derived outputs and control allocation --------------------------
function sample = sample_state(state, model)
qT = state(1:4) / norm(state(1:4));
omegaT = state(5:7);
z = state(8:10);

% Vane deflection and RCD modulation are states because the actuators have first-order dynamics.
deltaT = state(11:14);
deltaRhoT = state(15:18);

qe = quatmul(qT, quatinv(model.derived.qd));
if qe(4) < 0
    qe = -qe;
end

bodyDcm = quat2dcm(qT);
thetaE321 = dcm2euler321(quat2dcm(qe));
thetaT321 = dcm2euler321(bodyDcm);
alphaT = acos(max(-1, min(1, bodyDcm(1, 1))));
cosAlphaT = bodyDcm(1, 1);
cosAlphaIll = max(0, cosAlphaT);
alphaCos2 = cosAlphaIll ^ 2;

% Quaternion PID torque command.
qeVec = qe(1:3);
Tc = -model.derived.Kp * qeVec - model.derived.Ki * z - model.derived.Kd * omegaT + cross(omegaT, model.derived.inertia * omegaT);

% Pitch demand is shared between vane and RCD authority.
gammaY = 1 - (1 - model.controller.gamma0) * min(1, sin(alphaT) / sin(model.controller.atrans));
TyV = (1 - gammaY) * Tc(2); % Vane pitch control allocation
TyR = gammaY * Tc(2);       % RCD pitch control allocation
TxV = Tc(1);                % Vane roll control allocation
TzV = 0;                    % vane yaw authority is not used (Sec. 3.2.3)
TzR = Tc(3) - TzV;          % RCD yaw control allocation

% Vane command definitions, Sec. 2.4.4 Eq. (23):
%     delta_rc = Tx / (Fc L cos^2(alpha))
%     delta_pc = Ty / (4 Fc L cos^2(alpha) sin(alpha))
vaneGain = model.derived.vaneForce * model.derived.vaneMomentArm * alphaCos2;
deltaRc = TxV / max(vaneGain, model.controller.esing);
deltaPc = TyV / (4 * vaneGain * sin(alphaT) + model.controller.esing);

%     delta_1c = +delta_rc/2 , delta_4c = -delta_rc/2 , delta_2c = -delta_pc , delta_3c = +delta_pc
deltaTcmd = [ 0.5 * deltaRc; -deltaPc; deltaPc; -0.5 * deltaRc ];

DeltaCmd = deltaRc;         % retained for the recorded output
ThetaCmd = 0;               % common mode is unused, delta_4c = -delta_1c

% The commanded deflections may never exceed the mechanical stops
% |delta_rc| <= delta_rc,max and |delta_pc| <= delta_pc,max (Sec. 2.4.7).
for vaneIndex = 1:4
    if deltaTcmd(vaneIndex) > model.vanes.deflectionLimitVector(vaneIndex)
        deltaTcmd(vaneIndex) = model.vanes.deflectionLimitVector(vaneIndex);
    elseif deltaTcmd(vaneIndex) < -model.vanes.deflectionLimitVector(vaneIndex)
        deltaTcmd(vaneIndex) = -model.vanes.deflectionLimitVector(vaneIndex);
    end
end

% The RCD command comes from the pitch/yaw allocation inverse.
kr = model.environment.Psrp * model.rcd.quadrantArea * alphaCos2 * model.rcd.centroidOffset;
deltaRhoTcmd = (1 / max(4 * kr, model.controller.esing)) * [
    -TyR - TzR;
     TyR - TzR;
     TyR + TzR;
    -TyR + TzR];

% The commanded reflectivity increments may never exceed the optical limit
% |Delta rho_i| <= Delta rho_max (Sec. 2.5.5).
for quadrantIndex = 1:4
    if deltaRhoTcmd(quadrantIndex) > model.rcd.reflectivityLimit
        deltaRhoTcmd(quadrantIndex) = model.rcd.reflectivityLimit;
    elseif deltaRhoTcmd(quadrantIndex) < -model.rcd.reflectivityLimit
        deltaRhoTcmd(quadrantIndex) = -model.rcd.reflectivityLimit;
    end
end

% Actual torques use the realized actuator states, not the commands.
cosAlphaD2 = max(0, cos(alphaT - deltaT(2)));
cosAlphaD3 = max(0, cos(alphaT - deltaT(3)));
Tvanes = [
    model.derived.vaneForce * model.derived.vaneMomentArm * alphaCos2 * (cos(deltaT(1)) ^ 2 * sin(deltaT(1)) - cos(deltaT(4)) ^ 2 * sin(deltaT(4)));
    model.derived.vaneForce * model.derived.vaneMomentArm * (-cosAlphaD2 ^ 2 * cos(deltaT(2)) + cosAlphaD3 ^ 2 * cos(deltaT(3)));
   -model.derived.vaneForce * model.derived.vaneMomentArm * alphaCos2 * (cos(deltaT(1)) ^ 3 - cos(deltaT(4)) ^ 3)];

% RCD torques from reflectivity modulation (pitch and yaw only)
Trcd = [
    0;                                          % Roll torque: zero from RCD
    model.rcd.centroidOffset * model.environment.Psrp * model.rcd.quadrantArea * alphaCos2 * (deltaRhoT(2) + deltaRhoT(3) - deltaRhoT(1) - deltaRhoT(4));  % Pitch
    model.rcd.centroidOffset * model.environment.Psrp * model.rcd.quadrantArea * alphaCos2 * (deltaRhoT(3) + deltaRhoT(4) - deltaRhoT(2) - deltaRhoT(1))];  % Yaw

Tdist = [0; 0; 0];
if model.disturbance.enable
    Tdist(3) = model.disturbance.cmCpOffset * model.derived.solarForce * alphaCos2;
end

sample.attitudeError321 = thetaE321;
sample.absoluteEuler321 = thetaT321;
sample.sunAngle = alphaT;
sample.qeVec = qeVec;
sample.commandTorque = Tc;
sample.deltaTcmd = deltaTcmd;
sample.deltaRhoTcmd = deltaRhoTcmd;
sample.gammaY = gammaY;
sample.ThetaCmd = ThetaCmd;
sample.DeltaCmd = DeltaCmd;
sample.Tvanes = Tvanes;
sample.Trcd = Trcd;
sample.TvanesCmd = [TxV; TyV; TzV];
sample.TrcdCmd = [0; TyR; TzR];
sample.Tdist = Tdist;
sample.Tactuator = Tvanes + Trcd;
sample.Ttotal = sample.Tactuator + Tdist;
end

% --- Utility helpers --------------------------------------------------
function quaternionHistory = normalize_quaternion_history(quaternionHistory)
for rowIndex = 1:size(quaternionHistory, 1)
    quaternionHistory(rowIndex, :) = quaternionHistory(rowIndex, :) / norm(quaternionHistory(rowIndex, :));
end
end

function y = sat(u, limit)
y = max(-limit, min(limit, u));
end

function S = skew(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function C = C1(angle)
c = cos(angle);
s = sin(angle);
C = [1 0 0; 0 c s; 0 -s c];
end

function C = C2(angle)
c = cos(angle);
s = sin(angle);
C = [c 0 -s; 0 1 0; s 0 c];
end

function C = C3(angle)
c = cos(angle);
s = sin(angle);
C = [c s 0; -s c 0; 0 0 1];
end

function C = quat2dcm(q)
e = q(1:3);
n = q(4);
C = (n ^ 2 - e.' * e) * eye(3) + 2 * (e * e.') - 2 * n * skew(e);
end

function q = dcm2quat(C)
traceValue = trace(C);
if traceValue > 0
    n = 0.5 * sqrt(1 + traceValue);
    e = [C(2, 3) - C(3, 2); C(3, 1) - C(1, 3); C(1, 2) - C(2, 1)] / (4 * n);
else
    [~, i] = max([C(1, 1) C(2, 2) C(3, 3)]);
    j = mod(i, 3) + 1;
    k = mod(i + 1, 3) + 1;
    e = zeros(3, 1);
    e(i) = 0.5 * sqrt(1 + C(i, i) - C(j, j) - C(k, k));
    e(j) = (C(i, j) + C(j, i)) / (4 * e(i));
    e(k) = (C(i, k) + C(k, i)) / (4 * e(i));
    n = (C(j, k) - C(k, j)) / (4 * e(i));
end
q = [e; n] / norm([e; n]);
if q(4) < 0
    q = -q;
end
end

function q = quatmul(qa, qb)
ea = qa(1:3);
na = qa(4);
eb = qb(1:3);
nb = qb(4);
q = [na * eb + nb * ea - cross(ea, eb); na * nb - ea.' * eb];
end

function q = quatinv(q)
q = [-q(1:3); q(4)];
end

function euler321 = dcm2euler321(C)
pitch = -asin(max(-1, min(1, C(1, 3))));
yaw = atan2(C(1, 2), C(1, 1));
roll = atan2(C(2, 3), C(3, 3));
euler321 = [roll; pitch; yaw];
end