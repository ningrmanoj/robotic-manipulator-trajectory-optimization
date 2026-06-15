% MATLAB Script: Time-Optimal Trajectory with Torque Constraints (Single File)

clear; clc;

%% ================================
%% 1. USER INPUTS
%% ================================
N = 100;
num_splines = 6;

theta_start = [-2.09, 0];
theta_end   = [1.01, 0.26];
v_start = [0, 0];
v_end   = [0, 0];

% Initial Guess (z)
z0 = [-1.573, -1.057, -0.540, -0.023, 0.493, ...
       0.043, 0.087, 0.130, 0.173, 0.217, ...
       1.05];

% Torque limits (NOW THESE WILL BE USED PROPERLY)
tau1_max = 0.7;
tau2_max = 0.207;

%% ================================
%% 2. OPTIMIZATION SETUP
%% ================================
lb = [];
ub = [];

% Optional (recommended)
lb = [-inf*ones(1,10), 0.2];  % tf >= 0.2

options = optimoptions('fmincon', ...
    'Display','iter', ...
    'Algorithm','sqp', ...
    'MaxFunctionEvaluations', 1e5);

%% ================================
%% 3. RUN OPTIMIZATION
%% ================================
z_opt = fmincon(@obj_fun, z0, [], [], [], [], lb, ub, ...
    @(z) nonlcon(z, tau1_max, tau2_max), options);

disp('Optimized z:');
disp(z_opt);

%% ================================
%% 4. FINAL TRAJECTORY + TORQUE
%% ================================
[traj_matrix, tau_matrix, t_query] = compute_all(z_opt);

%% ================================
%% 5. DISPLAY TABLE
%% ================================
ResultTable = array2table([t_query, traj_matrix(:,2:7), tau_matrix], ...
    'VariableNames', {'Time','J1_Pos','J1_Vel','J1_Acc', ...
                      'J2_Pos','J2_Vel','J2_Acc','Tau1','Tau2'});

openvar('ResultTable');

%% ================================
%% NORMALIZE TIME AXIS (0 to 1)
%% ================================
t_norm = (t_query - t_query(1)) / (t_query(end) - t_query(1));

%% ================================
%% 6. PLOTS (TIME NORMALIZED)
%% ================================

%% -------- FIGURE 1: TORQUE --------
figure('Name','Joint Torque Profiles','NumberTitle','off');

subplot(2,1,1);
plot(t_norm, tau_matrix(:,1), 'LineWidth',1.5);
title('Joint 1 Torque');
ylabel('Torque (Nm)');
grid on;

subplot(2,1,2);
plot(t_norm, tau_matrix(:,2), 'LineWidth',1.5);
title('Joint 2 Torque');
ylabel('Torque (Nm)');
xlabel('Normalized Time (0–1)');
grid on;

%% -------- FIGURE 2: MOTION --------
figure('Name','Joint Motion Profiles','NumberTitle','off');

% --- POSITION ---
subplot(3,1,1);
plot(t_norm, traj_matrix(:,[2 5]), 'LineWidth',1.5);
title('Joint Positions');
ylabel('rad');
legend('J1','J2');
grid on;

% --- VELOCITY ---
subplot(3,1,2);
plot(t_norm, traj_matrix(:,[3 6]), 'LineWidth',1.5);
title('Joint Velocities');
ylabel('rad/s');
legend('J1','J2');
grid on;

% --- ACCELERATION ---
subplot(3,1,3);
plot(t_norm, traj_matrix(:,[4 7]), 'LineWidth',1.5);
title('Joint Accelerations');
ylabel('rad/s^2');
xlabel('Normalized Time (0–1)');
legend('J1','J2');
grid on;

disp('Optimization complete. Time-normalized plots generated.');

%% =========================================================
%% ================= LOCAL FUNCTIONS ========================
%% =========================================================

function f = obj_fun(z)
    % Objective: minimize final time
    f = z(end);
end

function [c, ceq] = nonlcon(z, tau1_max, tau2_max)

    [~, tau_matrix, ~] = compute_all(z);

    tau1 = tau_matrix(:,1);
    tau2 = tau_matrix(:,2);

    % Use INPUT torque limits
    c = [abs(tau1) - tau1_max;
         abs(tau2) - tau2_max];

    ceq = [];
end

function [traj_matrix, tau_matrix, t_query] = compute_all(z)

    %% Access shared variables
    N = evalin('base','N');
    num_splines = evalin('base','num_splines');
    theta_start = evalin('base','theta_start');
    theta_end   = evalin('base','theta_end');
    v_start     = evalin('base','v_start');
    v_end       = evalin('base','v_end');

    %% Extract tf + via points
    tf = z(end);
    num_via = num_splines - 1;

    joint1_via = z(1:num_via);
    joint2_via = z(num_via+1 : 2*num_via);
    via_points = [joint1_via(:), joint2_via(:)];

    %% Physical parameters
    g = 9.81;
    m1 = 0.193; m2 = 0.115;
    l1 = 0.25;
    lc1 = 0.198; lc2 = 0.143;
    I1 = 0.001149; I2 = 0.0004993;

    %% Trajectory
    t_knots = linspace(0, tf, num_splines + 1);
    all_thetas = [theta_start; via_points; theta_end];
    t_query = linspace(0, tf, N)';

    traj_matrix = zeros(N,7);
    traj_matrix(:,1) = t_query;

    for j = 1:2
        pp = spline(t_knots, [v_start(j), all_thetas(:,j)', v_end(j)]);

        traj_matrix(:, (j-1)*3 + 2) = ppval(pp, t_query);

        [breaks, coefs, ~, ~, d] = unmkpp(pp);
        v_coefs = [3*coefs(:,1), 2*coefs(:,2), coefs(:,3)];
        pp_vel = mkpp(breaks, v_coefs, d);
        traj_matrix(:, (j-1)*3 + 3) = ppval(pp_vel, t_query);

        a_coefs = [2*v_coefs(:,1), v_coefs(:,2)];
        pp_accel = mkpp(breaks, a_coefs, d);
        traj_matrix(:, (j-1)*3 + 4) = ppval(pp_accel, t_query);
    end

    %% Torque computation
    tau_matrix = zeros(N,2);

    for i = 1:N
        q  = [traj_matrix(i,2); traj_matrix(i,5)];
        dq = [traj_matrix(i,3); traj_matrix(i,6)];
        ddq= [traj_matrix(i,4); traj_matrix(i,7)];

        M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q(2)));
        M12 = I2 + m2*(lc2^2 + l1*lc2*cos(q(2)));
        M22 = I2 + m2*lc2^2;
        M = [M11, M12; M12, M22];

        h = m2 * l1 * lc2 * sin(q(2));
        C = [-h*dq(2), -h*(dq(1)+dq(2));
              h*dq(1), 0];

        G1 = (m1*lc1 + m2*l1)*g*cos(q(1)) + m2*lc2*g*cos(q(1)+q(2));
        G2 = m2*lc2*g*cos(q(1)+q(2));
        G = [G1; G2];

        tau = M*ddq + C*dq + G;

        tau_matrix(i,:) = tau';
    end
end