% MATLAB Script: Full 100-Point Trajectory Grid with Visualization
% No Toolboxes Required

%% 1. User Inputs (Modified Format)

N = 100;                    
num_splines = 6;            

theta_start = [-2.09, 0];   
theta_end = [1.01, 0.26];   
v_start = [0, 0];           
v_end = [0, 0];             

% New compact input vector
z = [-1.573, -1.057, -0.540, -0.023, 0.493, ...
      0.043, 0.087, 0.130, 0.173, 0.217, ...
      1.05];

% --- Extract tf ---
tf = z(end);

% --- Extract via points ---
num_via = num_splines - 1;  % 5 internal knots

joint1_via = z(1:num_via);
joint2_via = z(num_via+1 : 2*num_via);

via_points = [joint1_via(:), joint2_via(:)];

%% 2. Setup Vectors
t_knots = linspace(0, tf, num_splines + 1); 
all_thetas = [theta_start; via_points; theta_end];
t_query = linspace(0, tf, N)'; 

%% 3. Generate Trajectory Matrix (Time + Pos/Vel/Accel for 2 joints)
traj_matrix = zeros(N, 7);
traj_matrix(:, 1) = t_query;

for j = 1:2
    % Generate Cubic Spline
    pp = spline(t_knots, [v_start(j), all_thetas(:,j)', v_end(j)]);
    
    % --- Position ---
    traj_matrix(:, (j-1)*3 + 2) = ppval(pp, t_query);
    
    % --- Velocity (Derivative 1) ---
    [breaks, coefs, l, k, d] = unmkpp(pp);
    v_coefs = [3*coefs(:,1), 2*coefs(:,2), coefs(:,3)];
    pp_vel = mkpp(breaks, v_coefs, d);
    traj_matrix(:, (j-1)*3 + 3) = ppval(pp_vel, t_query);
    
    % --- Acceleration (Derivative 2) ---
    a_coefs = [2*v_coefs(:,1), v_coefs(:,2)]; 
    pp_accel = mkpp(breaks, a_coefs, d);
    traj_matrix(:, (j-1)*3 + 4) = ppval(pp_accel, t_query);
end

%% 4. Create Table and Open Grid View
TrajectoryTable = array2table(traj_matrix, 'VariableNames', ...
    {'Time_s', 'J1_Pos', 'J1_Vel', 'J1_Acc', 'J2_Pos', 'J2_Vel', 'J2_Acc'});

openvar('TrajectoryTable'); 

%% 5. Generate Graphs
figure('Name', '2R Manipulator Joint Space Trajectory', 'NumberTitle', 'off');

% Position Plot
subplot(3,1,1);
plot(t_query, traj_matrix(:, [2, 5]), 'LineWidth', 1.5);
title('Joint Positions'); ylabel('Angle (rad)');
legend('Joint 1', 'Joint 2'); grid on;

% Velocity Plot
subplot(3,1,2);
plot(t_query, traj_matrix(:, [3, 6]), 'LineWidth', 1.5);
title('Joint Velocities'); ylabel('Vel (rad/s)');
legend('Joint 1', 'Joint 2'); grid on;

% Acceleration Plot
subplot(3,1,3);
plot(t_query, traj_matrix(:, [4, 7]), 'LineWidth', 1.5);
title('Joint Accelerations'); ylabel('Acc (rad/s^2)'); xlabel('Time (s)');
legend('Joint 1', 'Joint 2'); grid on;

disp('Trajectory generation complete. Grid view and graphs are now open.');
