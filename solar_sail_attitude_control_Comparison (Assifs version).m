function solar_sail_attitude_control_Comparison
%SOLAR_SAIL_ATTITUDE_CONTROL
%   Closed-loop simulation of the hybrid vane / RCD attitude control system
%   described in "Solar-Sail Dynamics and Control", 086926 final project.
%
%   The script runs FOUR cases and overlays them, one figure per body axis:
%
%     1. no disturbance ,  vane yaw authority ON
%     2. no disturbance ,  vane yaw authority OFF
%     3. eps*F yaw disturbance , vane yaw authority ON
%     4. eps*F yaw disturbance , vane yaw authority OFF
%
%   "Vane yaw authority" means the second-order term
%        Tz_vane = -Fc*L*cos^2(a)*(cos^3(d1) - cos^3(d4)) ~ 1.5*Fc*L*cos^2(a)*Delta*Theta
%   is EXPLOITED by the control allocation, through a non-zero common-mode
%   deflection Theta = d1 + d4.  When it is OFF the allocation sets Theta = 0
%   and yaw is left entirely to the RCD quadrants.  In BOTH cases the plant
%   integrates the exact non-linear vane torques, so nothing is hidden.
%
%   Runs unmodified in MATLAB and in GNU Octave.
%
%   Outputs:  fig_roll.png, fig_pitch.png, fig_yaw.png,
%             fig_absolute.png, fig_actuators.png, fig_pd_vs_pid.png
%
%   ---------------------------------------------------------------------
%   NOTE ON TWO CORRECTED COEFFICIENTS (see report Sec. 2.5.4 / 2.5.7):
%     * Eq. (pitch Taylor)  d/dd[cos^2(a-d)cos(d)]|_0 = sin(2a) = 2*cos(a)*sin(a),
%       NOT 2*cos^2(a)*sin(a).  Hence  Ty ~ 4*Fc*L*cos(a)*sin(a)*dp.
%     * Eq. (theta_cmd) Theta = Tz/Tx is SINGULAR at Tx = 0.  Used literally it
%       makes the roll/yaw vanes slam between +-Theta_max every time the roll
%       torque changes sign.  A damped-least-squares inverse is used instead.
%     * Eq. (vane_cmds) d4 = (Theta-Delta)/2, NOT (Delta-Theta)/2; as written the
%       two roll/yaw vanes deflect together and ALL roll authority is lost.
%     * Eq. (yaw 2nd order) cos^3(d) ~ 1 - (3/2)d^2, hence
%       Tz ~ (3/2)*Fc*L*cos^2(a)*Delta*Theta  and  Theta = (2/3)*Tz/Tx.
%   All are verified numerically by SELFTEST below.
%
%   CONTROLLER: quaternion PID (report Sec. 3.2.2).  Set P.useI = false for the
%   pure-PD law; the eps*F disturbance then leaves a 6.28 deg yaw droop.
%   ---------------------------------------------------------------------

clc;
if exist('OCTAVE_VERSION','builtin'); pkg_quiet(); end

selftest();

P = params();

cases = struct( ...
  'name' ,{'no dist., vane yaw ON','no dist., vane yaw OFF', ...
           'eps F dist., vane yaw ON','eps F dist., vane yaw OFF'}, ...
  'dist' ,{false,false,true,true}, ...
  'yaw'  ,{true ,false,true,false}, ...
  'col'  ,{[0 0.45 0.74],[0.85 0.33 0.10],[0.20 0.62 0.20],[0.60 0.20 0.65]}, ...
  'ls'   ,{'-','--','-','--'} );

R = cell(1,numel(cases));
for k = 1:numel(cases)
    Pk = P;  Pk.useDist = cases(k).dist;  Pk.useVaneYaw = cases(k).yaw;
    fprintf('running case %d/%d : %s\n',k,numel(cases),cases(k).name);
    R{k} = simulate(Pk);
end

make_plots(R,cases,P);

% ---- PD vs PID on the axis that carries the disturbance (yaw) ------------
Rpd = cell(1,2);
for k = 1:2
    Pk = params();  Pk.useI = false;  Pk = regains(Pk);
    Pk.useDist = true;  Pk.useVaneYaw = (k==1);
    fprintf('running PD reference %d/2\n',k);
    Rpd{k} = simulate(Pk);
end
plot_pd_vs_pid(R,Rpd,cases,P);

report(R,cases,P);
end
% =======================================================================
%                              PARAMETERS
% =======================================================================
function P = params()
% ---- environment ------------------------------------------------------
P.Psrp   = 4.563e-6;          % N/m^2   SRP constant at 1 AU      (Sec. 1.1)
P.eta    = 1.8;               % -       sail thrust efficiency    (Sec. 1.1)
P.alpha0 = atan(1/sqrt(2));   % rad     alpha* = 35.264 deg       (Sec. 1.2)
P.alphaCmd = P.alpha0;        % rad  ** COMMANDED PITCH (sun) ANGLE = alpha* **
                              %      The target frame is the sun-line frame
                              %      pitched by alphaCmd about +j, so at trim
                              %      theta_2^abs = -alphaCmd and alpha = alphaCmd.

% ---- sail / spacecraft ------------------------------------------------
P.aSail  = 40;                % m       square sail side
P.Asail  = P.aSail^2;         % m^2
P.L      = P.aSail/sqrt(2);   % m       spar length = half diagonal = 28.28 m
P.I      = diag([16000 8000 8000]);   % kg m^2  flat plate: Ix = Iy + Iz
P.Iinv   = inv(P.I);

% ---- control vanes ----------------------------------------------------
P.Ac     = 25;                % m^2     area of ONE vane
P.Fc     = 2.0*P.Psrp*P.Ac;   % N       max SRP force on one vane (eta_max = 2)
P.dmax   = deg2rad(45);       % rad     vane deflection limit
P.dratemx= deg2rad(5);        % rad/s   vane slew-rate limit
P.tauV   = 10;                 % s       vane servo time constant
P.Thmax  = deg2rad(40);       % rad     limit on common mode Theta = d1 + d4
P.muTx   = 5e-4;              % N m     damped-least-squares regularisation of the
                              %         Theta = (2/3)Tz/Tx inversion.  The RAW law
                              %         is singular at Tx = 0 and makes the vanes
                              %         chatter between +-Thmax; see notes.

% ---- reflectivity control devices ------------------------------------
P.Aq     = 40;                % m^2     RCD-COVERED area per quadrant
                              %         (= 10 % of a 400 m^2 sail quadrant)
P.rq     = 10;                % m       quadrant centroid offset
P.drhomx = 0.30;              % -       reflectivity modulation limit
P.tauR   = 20;                % s       electrochromic transition lag

% ---- disturbance ------------------------------------------------------
P.epsCP  = 0.10;              % m       cm / cp offset along +y
P.Fsrp   = P.eta*P.Psrp*P.Asail;      % N   nominal SRP force

% ---- controller -------------------------------------------------------
P.wn     = [1.0e-3 1.0e-3 1.0e-3];    % rad/s dominant natural frequency
P.zeta   = [0.80 0.80 0.80];          % -     dominant damping ratio
P.wi     = P.wn/4;                    % rad/s real integrator pole
P.useI   = true;                      % false -> pure PD (the old behaviour)
P.Tintmx = 2.5e-3;            % N m  anti-windup cap on the integral torque
P = regains(P);
P.gamma0 = 0.10;              % -   nominal RCD share of pitch
P.atrans = deg2rad(15);       % rad transition angle of the pitch gain schedule
P.esing  = 1e-9;              % numerical regularisation of the pitch inversion

% ---- simulation -------------------------------------------------------
P.tEnd   = 30000;             % s
P.dt     = 4.0;               % s   fixed-step RK4
P.th0    = deg2rad([5 -4 10]);% rad initial 3-2-1 attitude ERROR (roll,pitch,yaw)
P.w0     = [0;0;0];           % rad/s
end
% -----------------------------------------------------------------------
function P = regains(P)
%REGAINS  gain selection by pole placement.
%   Linearised axis (eps_e ~ theta/2):
%       I th'' + kd th' + (kp/2) th + (ki/2)*int(th) = T_dist
%   Place the roots of  (s^2 + 2 zeta wn s + wn^2)(s + wi) = 0 :
Ivec = diag(P.I).';
if P.useI
    P.Kd = diag( Ivec.*(2*P.zeta.*P.wn + P.wi) );
    P.Kp = diag( 2*Ivec.*(P.wn.^2 + 2*P.zeta.*P.wn.*P.wi) );
    P.Ki = diag( 2*Ivec.*P.wn.^2.*P.wi );
else                                   % PD of report Sec. 3.3.2
    P.Kp = diag( 2*Ivec.*P.wn.^2 );
    P.Kd = diag( 2*P.zeta.*P.wn.*Ivec );
    P.Ki = zeros(3);
end
P.zmax = P.Tintmx./max(diag(P.Ki).',1e-30);
end
% =======================================================================
%                              SIMULATION
% =======================================================================
function R = simulate(P)
Cd  = C2(-P.alphaCmd);               % target DCM  (body <- sun-line frame)
P.qd = dcm2quat(Cd);
Ce0 = C1(P.th0(1))*C2(P.th0(2))*C3(P.th0(3));
q0  = dcm2quat(Ce0*Cd);

x = [q0; P.w0; zeros(3,1); zeros(4,1); zeros(4,1)];   % 18 states
%     q(1:4)  w(5:7)  z(8:10)  delta(11:14)  drho(15:18)
N = round(P.tEnd/P.dt);
t = (0:N).'*P.dt;

nx = numel(x);
X  = zeros(N+1,nx);  X(1,:) = x.';
TH = zeros(N+1,3);   AL = zeros(N+1,1);   TA = zeros(N+1,3);
[TH(1,:),AL(1),TA(1,:)] = outputs(x,P);

for k = 1:N                                   % classical RK4
    k1 = deriv(x           ,P);
    k2 = deriv(x+0.5*P.dt*k1,P);
    k3 = deriv(x+0.5*P.dt*k2,P);
    k4 = deriv(x+    P.dt*k3,P);
    x  = x + (P.dt/6)*(k1+2*k2+2*k3+k4);
    x(1:4) = x(1:4)/norm(x(1:4));             % unit-norm projection
    X(k+1,:) = x.';
    [TH(k+1,:),AL(k+1),TA(k+1,:)] = outputs(x,P);
end

R.t = t;  R.X = X;  R.th = TH;  R.alpha = AL;  R.thabs = TA;
R.delta = X(:,11:14); R.drho = X(:,15:18); R.z = X(:,8:10);
end
% -----------------------------------------------------------------------
function [th,al,tha] = outputs(x,P)
q  = x(1:4);  Cb = quat2dcm(q);
qe = quatmul(q, quatinv(P.qd));
if qe(4) < 0, qe = -qe; end
th  = dcm2euler321(quat2dcm(qe)).';   % ERROR angles  (B relative to target)
tha = dcm2euler321(Cb).';             % ABSOLUTE angles (B relative to sun-line frame)
al  = acos(max(-1,min(1,Cb(1,1))));
end
% -----------------------------------------------------------------------
function xdot = deriv(x,P)
q  = x(1:4)/norm(x(1:4));
w  = x(5:7);
z  = x(8:10);                        % integral of the error-quaternion vector
d  = x(11:14);                       % realised vane angles  [d1 d2 d3 d4]
dr = x(15:18);                       % realised d(rho)       [TR TL BL BR]

[~,dc,drc,info] = controller(x,P);

% ---- plant: true sun angle -------------------------------------------
Cb = quat2dcm(q);
aT = acos(max(-1,min(1,Cb(1,1))));   % alpha_T = acos(-S_b . i)

% ---- exact vane torques (report Eqs. exact_roll / pitch / yaw) --------
ca2 = cos(aT)^2;
Tv  = [ P.Fc*P.L*ca2*( cos(d(1))^2*sin(d(1)) - cos(d(4))^2*sin(d(4)) );
        P.Fc*P.L*( -cos(aT-d(2))^2*cos(d(2)) + cos(aT-d(3))^2*cos(d(3)) );
       -P.Fc*P.L*ca2*( cos(d(1))^3 - cos(d(4))^3 ) ];

% ---- RCD torques ------------------------------------------------------
kq = P.Psrp*P.Aq*ca2;
Tr = [ 0;
       P.rq*kq*( dr(2) + dr(3) - dr(1) - dr(4) );
       P.rq*kq*( dr(3) + dr(4) - dr(2) - dr(1) ) ];

% ---- cm/cp disturbance (pure yaw) ------------------------------------
Td = [0;0;0];
if P.useDist
    Td(3) = P.epsCP*P.Fsrp*ca2;
end

Ttot = Tv + Tr + Td;

% ---- rigid-body dynamics + quaternion kinematics ----------------------
wdot = P.Iinv*( -cross(w,P.I*w) + Ttot );
eps_ = q(1:3);  eta_ = q(4);
epsd = 0.5*( eta_*eye(3) + skew(eps_) )*w;
etad = -0.5*(eps_.'*w);

% ---- actuator dynamics + saturation ----------------------------------
ddot = sat( (sat(dc,P.dmax) - d)/P.tauV , P.dratemx );
drdot= ( sat(drc,P.drhomx) - dr )/P.tauR;

% ---- integrator with conditional-integration anti-windup --------------
%  freeze the axis whose integral state is already at its limit and whose
%  error would push it further out.
zdot = info.epse;
for i = 1:3
    if abs(z(i)) >= P.zmax(i) && sign(zdot(i)) == sign(z(i))
        zdot(i) = 0;
    end
end
if ~P.useI, zdot = zeros(3,1); end

xdot = [epsd; etad; wdot; zdot; ddot; drdot];
end
% =======================================================================
%                        CONTROLLER + ALLOCATION
% =======================================================================
function [Tc,dc,drc,info] = controller(x,P)
q = x(1:4)/norm(x(1:4));   w = x(5:7);   z = x(8:10);

% ---- attitude error ---------------------------------------------------
qe = quatmul(q, quatinv(P.qd));
if qe(4) < 0, qe = -qe; end               % unwinding protection
epse = qe(1:3);

% ---- outer-loop quaternion PID with gyroscopic decoupling -------------
%   T_c = -Kp*eps_e - Ki*z - Kd*w_e + w x I w ,     zdot = eps_e
Tc = -P.Kp*epse - P.Ki*z - P.Kd*w + cross(w,P.I*w);

% ---- measured sun angle ----------------------------------------------
Cb = quat2dcm(q);
am = acos(max(-1,min(1,Cb(1,1))));
cam2 = cos(am)^2;
FcLc = P.Fc*P.L*cam2;                     % roll/yaw vane gain

% ---- pitch split, gain-scheduled on the sun angle ---------------------
gy   = 1 - (1-P.gamma0)*min(1, sin(am)/sin(P.atrans));
TyV  = (1-gy)*Tc(2);
TyR  =    gy *Tc(2);

% ---- roll: vanes only -------------------------------------------------
TxV  = Tc(1);
Delta= TxV/FcLc;

% ---- yaw --------------------------------------------------------------
%  Tz_vane = (3/2) Fc L cos^2(a) * Delta * Theta = (3/2) TxV * Theta
Theta = 0;  TzV = 0;
if P.useVaneYaw
    % Damped least squares instead of the raw Theta = (2/3)Tz/Tx :
    %    Theta = (2/3) Tz Tx /(Tx^2 + mu^2)
    % identical to the report's law when |Tx| >> mu, but rolls off smoothly to
    % zero as the roll loop converges and Tx -> 0, instead of diverging.
    Theta = (2/3)*Tc(3)*TxV/(TxV^2 + P.muTx^2);
    Theta = max(-P.Thmax, min(P.Thmax, Theta));
    TzV   = 1.5*TxV*Theta;
end
TzR = Tc(3) - TzV;

% ---- vane commands ----------------------------------------------------
d1 = 0.5*( Theta + Delta);      % from Delta = d1-d4 , Theta = d1+d4
d4 = 0.5*( Theta - Delta);      % (the report writes d4 = (Delta-Theta)/2 -- sign slip)
dp = TyV/( 4*P.Fc*P.L*cos(am)*sin(am) + P.esing );   % corrected coefficient
dc = [d1; -dp; dp; d4];

% ---- RCD commands (pseudo-inverse) ------------------------------------
kr  = P.Psrp*P.Aq*cam2*P.rq;
drc = ( 1/(4*kr) )*[ -TyR - TzR ;
                      TyR - TzR ;
                      TyR + TzR ;
                     -TyR + TzR ];
info = struct('gamma',gy,'Theta',Theta,'Delta',Delta,'alpha',am,'epse',epse);
end
% =======================================================================
%                              PLOTTING
% =======================================================================
function make_plots(R,cases,P)
ax   = {'Roll  \theta_1','Pitch  \theta_2','Yaw  \theta_3'};
fn   = {'fig_roll.png','fig_pitch.png','fig_yaw.png'};
for i = 1:3
    f = figure('Visible','off','Position',[100 100 900 520]);
    hold on; grid on; box on;
    for k = 1:numel(R)
        plot(R{k}.t/3600, rad2deg(R{k}.th(:,i)), cases(k).ls, ...
             'Color',cases(k).col,'LineWidth',1.6);
    end
    plot([0 P.tEnd/3600],[0 0],'k:','LineWidth',0.8,'HandleVisibility','off');
    xlabel('time  [h]');
    ylabel([ax{i} ' attitude error  [deg]']);
    title(['Closed-loop attitude ERROR --- ' ax{i} ...
           sprintf('   (command: \\theta_2 = -%.2f deg, \\theta_1 = \\theta_3 = 0)', ...
                   rad2deg(P.alphaCmd))]);
    legend({cases.name},'Location','northeast');
    set(gca,'FontSize',11);
    print(f,'-dpng','-r150',fn{i});  close(f);
end

% --- absolute Euler angles, with the commanded values marked ----------
f = figure('Visible','off','Position',[100 100 900 820]);
cmdv = [0, -rad2deg(P.alphaCmd), 0];
nm   = {'roll  \theta_1','pitch  \theta_2','yaw  \theta_3'};
for i = 1:3
    subplot(3,1,i); hold on; grid on; box on;
    for k = 1:numel(R)
        plot(R{k}.t/3600, rad2deg(R{k}.thabs(:,i)), cases(k).ls, ...
             'Color',cases(k).col,'LineWidth',1.5);
    end
    plot([0 P.tEnd/3600], cmdv(i)*[1 1],'k--','LineWidth',1.2);
    ylabel([nm{i} '  [deg]']);
    if i==1
        title(sprintf(['Absolute attitude (body w.r.t. sun-line frame).  ' ...
                       'Dashed black = command:  0 / %.2f\\circ / 0'],cmdv(2)));
        legend([{cases.name},{'command'}],'Location','east');
    end
    if i==3, xlabel('time  [h]'); end
    set(gca,'FontSize',10);
end
print(f,'-dpng','-r150','fig_absolute.png'); close(f);

% --- extra diagnostic: actuator effort --------------------------------
f = figure('Visible','off','Position',[100 100 980 720]);
subplot(2,2,1); hold on; grid on; box on;
for k=1:numel(R), plot(R{k}.t/3600, rad2deg(R{k}.delta(:,1)),cases(k).ls,'Color',cases(k).col,'LineWidth',1.3); end
ylabel('\delta_1 [deg]'); title('roll/yaw vane 1'); xlabel('time [h]');
subplot(2,2,2); hold on; grid on; box on;
for k=1:numel(R), plot(R{k}.t/3600, rad2deg(R{k}.delta(:,4)),cases(k).ls,'Color',cases(k).col,'LineWidth',1.3); end
ylabel('\delta_4 [deg]'); title('roll/yaw vane 4'); xlabel('time [h]');
subplot(2,2,3); hold on; grid on; box on;
for k=1:numel(R), plot(R{k}.t/3600, R{k}.drho(:,3),cases(k).ls,'Color',cases(k).col,'LineWidth',1.3); end
plot([0 P.tEnd/3600], P.drhomx*[1 1],'k:','LineWidth',1);
plot([0 P.tEnd/3600],-P.drhomx*[1 1],'k:','LineWidth',1);
ylabel('\Delta\rho_{BL}'); title('RCD quadrant BL (dotted = saturation)'); xlabel('time [h]');
subplot(2,2,4); hold on; grid on; box on;
for k=1:numel(R), plot(R{k}.t/3600, rad2deg(R{k}.alpha),cases(k).ls,'Color',cases(k).col,'LineWidth',1.3); end
plot([0 P.tEnd/3600], rad2deg(P.alpha0)*[1 1],'k:','LineWidth',1);
ylabel('\alpha [deg]'); title('true sun angle'); xlabel('time [h]');
legend({cases.name},'Location','northeast');
print(f,'-dpng','-r150','fig_actuators.png'); close(f);
end
% -----------------------------------------------------------------------
function plot_pd_vs_pid(R,Rpd,cases,P)
f = figure('Visible','off','Position',[100 100 900 520]);
hold on; grid on; box on;
plot(Rpd{1}.t/3600, rad2deg(Rpd{1}.th(:,3)),'-' ,'Color',[0.85 0.33 0.10],'LineWidth',1.7);
plot(Rpd{2}.t/3600, rad2deg(Rpd{2}.th(:,3)),'--','Color',[0.85 0.33 0.10],'LineWidth',1.7);
plot(R{3}.t/3600  , rad2deg(R{3}.th(:,3))  ,'-' ,'Color',[0 0.45 0.74],'LineWidth',1.7);
plot(R{4}.t/3600  , rad2deg(R{4}.th(:,3))  ,'--','Color',[0 0.45 0.74],'LineWidth',1.7);
plot([0 P.tEnd/3600],[0 0],'k:','LineWidth',0.8,'HandleVisibility','off');
xlabel('time  [h]'); ylabel('yaw  \theta_3 attitude error  [deg]');
title('Effect of the integrator, \epsilon F disturbance active');
legend({'PD , vane yaw ON','PD , vane yaw OFF', ...
        'PID, vane yaw ON','PID, vane yaw OFF'},'Location','northeast');
set(gca,'FontSize',11);
print(f,'-dpng','-r150','fig_pd_vs_pid.png'); close(f);
end
% -----------------------------------------------------------------------
function report(R,cases,P)
fprintf('\n%-30s %9s %9s %9s %10s %10s\n','case','roll_ss','pitch_ss','yaw_ss','max|drho|','max|delta|');
fprintf('%s\n',repmat('-',1,85));
n = numel(R{1}.t);  idx = round(0.9*n):n;      % last 10 % of the run
for k = 1:numel(R)
    ss = rad2deg(mean(R{k}.th(idx,:),1));
    fprintf('%-30s %9.4f %9.4f %9.4f %10.4f %9.2f deg\n',cases(k).name, ...
            ss(1),ss(2),ss(3),max(abs(R{k}.drho(:))), ...
            rad2deg(max(max(abs(R{k}.delta)))));
end
fprintf('\ncommanded pitch (sun) angle  alpha_cmd        = %.4f deg\n',rad2deg(P.alphaCmd));
fprintf('initial attitude ERROR  (roll,pitch,yaw)      = [%.2f %.2f %.2f] deg\n',rad2deg(P.th0));
fprintf('initial ABSOLUTE angles (roll,pitch,yaw)      = [%.3f %.3f %.3f] deg\n',rad2deg(R{1}.thabs(1,:)));
fprintf('initial sun angle alpha(0)                    = %.3f deg\n',rad2deg(R{1}.alpha(1)));
fprintf('final   sun angle alpha(end), case 1          = %.3f deg\n',rad2deg(R{1}.alpha(end)));
fprintf('\nRCD yaw capacity   4*P*Aq*cos^2(a*)*r*drho_max = %.3e N m\n', ...
        4*P.Psrp*P.Aq*cos(P.alpha0)^2*P.rq*P.drhomx);
fprintf('cm/cp disturbance  eps*F*cos^2(a*)             = %.3e N m\n', ...
        P.epsCP*P.Fsrp*cos(P.alpha0)^2);
fprintf('yaw proportional gain kp_z                    = %.3e N m/rad\n',P.Kp(3,3));
fprintf('yaw integral     gain ki_z                    = %.3e N m/(rad s)\n',P.Ki(3,3));
fprintf('Routh margin  kd*kp / (I*ki)  (>1 = stable)   = %.2f\n', ...
        P.Kd(3,3)*P.Kp(3,3)/(P.I(3,3)*max(P.Ki(3,3),realmin)));
% PD droop: the P term is -kp*eps_e and eps_e ~ theta/2, so the effective
% stiffness seen by the angle is kp/2.
% With the PD gains of Sec. 3.3.2 (kp = 2 I wn^2) the droop is
%     theta_ss = T_dist/(kp/2) = T_dist/(I wn^2).   The integrator removes it.
fprintf('PD droop the integrator removes               = %.3f deg\n', ...
        rad2deg(P.epsCP*P.Fsrp*cos(P.alpha0)^2/(P.I(3,3)*P.wn(3)^2)));
end
% =======================================================================
%                          SMALL UTILITIES
% =======================================================================
function y = sat(u,lim),  y = max(-lim, min(lim,u));  end
function S = skew(v),     S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];  end
function C = C1(t), c=cos(t); s=sin(t); C=[1 0 0; 0 c s; 0 -s c]; end
function C = C2(t), c=cos(t); s=sin(t); C=[c 0 -s; 0 1 0; s 0 c]; end
function C = C3(t), c=cos(t); s=sin(t); C=[c s 0; -s c 0; 0 0 1]; end

function C = quat2dcm(q)      % q = [eps(3); eta],  C = C^{B/R}
e = q(1:3);  n = q(4);
C = (n^2 - e.'*e)*eye(3) + 2*(e*e.') - 2*n*skew(e);
end

function q = dcm2quat(C)
tr = trace(C);
if tr > 0
    n = 0.5*sqrt(1+tr);   e = [C(2,3)-C(3,2); C(3,1)-C(1,3); C(1,2)-C(2,1)]/(4*n);
else
    [~,i] = max([C(1,1) C(2,2) C(3,3)]);
    j = mod(i,3)+1;  k = mod(i+1,3)+1;
    e = zeros(3,1);
    e(i) = 0.5*sqrt(1 + C(i,i) - C(j,j) - C(k,k));
    e(j) = (C(i,j)+C(j,i))/(4*e(i));
    e(k) = (C(i,k)+C(k,i))/(4*e(i));
    n    = (C(j,k)-C(k,j))/(4*e(i));
end
q = [e; n];  q = q/norm(q);
if q(4) < 0, q = -q; end
end

function q = quatmul(qa,qb)   % composition matching quat2dcm: C(qa*qb)=C(qa)C(qb)
ea = qa(1:3); na = qa(4);  eb = qb(1:3); nb = qb(4);
q  = [ na*eb + nb*ea - cross(ea,eb) ; na*nb - ea.'*eb ];
end

function q = quatinv(qa), q = [-qa(1:3); qa(4)]; end

function th = dcm2euler321(C)  % C = C1(th1) C2(th2) C3(th3)
th2 = -asin( max(-1,min(1,C(1,3))) );
th3 =  atan2( C(1,2), C(1,1) );
th1 =  atan2( C(2,3), C(3,3) );
th  = [th1; th2; th3];
end

function pkg_quiet()
try, graphics_toolkit('gnuplot'); catch, end
try, warning('off','all'); catch, end
end
% =======================================================================
%                              SELF TEST
% =======================================================================
function selftest()
fprintf('--- self test ---\n');
tol = 1e-9;

% 1. quaternion <-> DCM round trip and the product convention
rng_state = 12345;  s = rng_state;
for m = 1:5
    s = mod(1103515245*s + 12345, 2^31);  a1 = (s/2^31-0.5)*2;
    s = mod(1103515245*s + 12345, 2^31);  a2 = (s/2^31-0.5)*2;
    s = mod(1103515245*s + 12345, 2^31);  a3 = (s/2^31-0.5)*2;
    Ca = C1(a1)*C2(a2)*C3(a3);
    s = mod(1103515245*s + 12345, 2^31);  b1 = (s/2^31-0.5)*2;
    Cb = C2(b1)*C3(a1);
    qa = dcm2quat(Ca); qb = dcm2quat(Cb);
    e1 = max(max(abs(quat2dcm(qa)-Ca)));
    e2 = max(max(abs(quat2dcm(quatmul(qa,qb)) - Ca*Cb)));
    e3 = max(abs(dcm2euler321(Ca) - [a1;a2;a3]));
    assert(e1<tol && e2<tol && e3<tol, 'quaternion convention test failed');
end
fprintf('  quat2dcm / dcm2quat / quatmul / euler321 : OK\n');

% 2. pitch-vane linearisation coefficient   d/dd[cos^2(a-d)cos d]|_0 = sin(2a)
a  = atan(1/sqrt(2));  h = 1e-6;
g  = @(d) cos(a-d)^2*cos(d);
num = (g(h)-g(-h))/(2*h);
fprintf('  d/dd[cos^2(a-d)cos d]|_0 : numeric %.9f   sin(2a) %.9f   2cos^2(a)sin(a) %.9f\n', ...
        num, sin(2*a), 2*cos(a)^2*sin(a));
assert(abs(num - sin(2*a)) < 1e-6, 'pitch coefficient test failed');

% 3. yaw second-order coefficient  cos^3(d) ~ 1 - 1.5 d^2
d = 1e-3;
fprintf('  (1-cos^3 d)/d^2          : numeric %.6f   expected 1.5\n',(1-cos(d)^3)/d^2);
assert(abs((1-cos(d)^3)/d^2 - 1.5) < 1e-3, 'yaw coefficient test failed');
fprintf('--- self test passed ---\n\n');
end
