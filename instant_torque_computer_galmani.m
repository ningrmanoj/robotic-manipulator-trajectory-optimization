% 2R Manipulator Torque Calculator (Inverse Dynamics)
clear; clc;

%% 1. STATIC PHYSICAL PARAMETERS (Input these once)
g = 9.81;                 % Gravity (m/s^2)
m1 = 0.193; m2 = 0.115;       % Mass of links (kg)
l1 = 0.25;                 % Length of Link 1 (m)
lc1 = 0.198; lc2 = 0.143;    % Distance to Center of Mass (m)
I1 = 0.001149; I2 = 0.0004993;     % Moments of Inertia (kg*m^2)

%% 2. DYNAMIC STATE VARIABLES (Inputs at a specific instant)
q = [-1; 0.2];         % Joint Positions [q1; q2] (rad)
dq = [2; 1];          % Joint Velocities [dq1; dq2] (rad/s)
ddq = [5; 2];         % Joint Accelerations [ddq1; ddq2] (rad/s^2)

%% 3. COMPUTE MATRICES

% Inertia Matrix M(q)
M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q(2)));
M12 = I2 + m2*(lc2^2 + l1*lc2*cos(q(2)));
M21 = M12;
M22 = I2 + m2*lc2^2;

M = [M11, M12; M21, M22];

% Coriolis/Centrifugal Matrix C(q, dq)
h = m2 * l1 * lc2 * sin(q(2));
C = [-h*dq(2), -h*(dq(1) + dq(2));
      h*dq(1),  0];

% Gravity Vector G(q)
G1 = (m1*lc1 + m2*l1)*g*cos(q(1)) + m2*lc2*g*cos(q(1) + q(2));
G2 = m2*lc2*g*cos(q(1) + q(2));

G = [G1; G2];

%% 4. SOLVE FOR TORQUE
% Equation: Tau = M*ddq + C*dq + G
tau = M*ddq + C*dq + G;

%% 5. DISPLAY RESULTS
fprintf('Calculated Torques at t=current:\n');
fprintf('Joint 1 Torque (Tau1): %.4f Nm\n', tau(1));
fprintf('Joint 2 Torque (Tau2): %.4f Nm\n', tau(2));
