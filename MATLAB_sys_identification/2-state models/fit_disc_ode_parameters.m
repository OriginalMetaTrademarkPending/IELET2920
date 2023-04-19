% We start by importing the discretized differential equations (via Forward
% Euler)
clear
type disc_diff_eq

% Next, we import the data retrieved from the system testing, as well as
% the starting points. For this we need the filepath where the readings
% are.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
FILEPATH = "../../python_scripts/test_bias12.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.topMid';
=======
FILEPATH = "../../python_scripts/test1.csv";
=======
FILEPATH = "../../python_scripts/test2.csv";
>>>>>>> 564e173 (Massive changes to code, and perhaps a new success???)
=======
FILEPATH = "../../python_scripts/test1.csv";
>>>>>>> cbd11b7 (good parameters)
=======
FILEPATH = "../../python_scripts/test3.csv";
>>>>>>> 61698af (Circuit and PCB)
=======
FILEPATH = "../../python_scripts/Estimate_R.csv";
>>>>>>> 63c81e0 (estimate of sensor meassurment)
=======
FILEPATH = "../../python_scripts/test6.csv";
>>>>>>> d0ba3a8 (estimate R changes)
=======
FILEPATH = "../../python_scripts/test6.csv";
>>>>>>> 4f56570 (Changes in the new estimator, will try to finalize tomorrow)
=======
FILEPATH = "../../python_scripts/test1.csv";
>>>>>>> beb4004 (Full Kalman Filter implementation in MATLAB)
=======
FILEPATH = "../../python_scripts/test1.csv";
>>>>>>> e10bbb0 (New parameter estimation changes)
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data1';
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
N = max(size(y_data));      %Number of samples to be registered
<<<<<<< HEAD
tspan = 180;                %Time span of the simulation in seconds
M_size = 100;
=======
tspan = 240;                %Time span of the simulation in seconds
<<<<<<< HEAD
>>>>>>> 564e173 (Massive changes to code, and perhaps a new success???)
=======
M_size = 50;
>>>>>>> e10bbb0 (New parameter estimation changes)
%% INITIALIZING SIMULATION
t_vec = linspace(0, tspan, N);      %Time vector for plotting and input generation

% Defining the phi parameters. These parameters are defined as the theta
% parameters adjusted for the sample time. These parameters must be within
% 0 and 1. The last parameter is the total muscle mass. This parameter does
<<<<<<< HEAD
<<<<<<< HEAD
% not need to be adjusted for the sample time, but will be included as a
% family of different parameters.
phi_first_guess = [0.5, 0.5, 0.5, 0.5]; 
M = linspace(3, 40, M_size);
=======
% not need to be adjusted for the sample time.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
phi_first_guess = [0.3, 0.7, 0.9, 0.3, 20];
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
=======
phi_first_guess = [0.9, 0.7, 0.6, 0.9, 20];
>>>>>>> cbd11b7 (good parameters)
=======
phi_first_guess = [0.99, 0.7, 0.6, 0.9, 20]; 
=======
phi_first_guess = [0.99, 0.7, 0.6, 0.9]; 

M = linspace(0,30,1000);
>>>>>>> 6d69de0 (M - changes)
=======
% not need to be adjusted for the sample time, but will be included as a
% family of different parameters.
phi_first_guess = [0.2, 0.5, 0.9, 0.7]; 
M = linspace(3, 30, 100);
>>>>>>> e10bbb0 (New parameter estimation changes)
% af, 
>>>>>>> 61698af (Circuit and PCB)

% The input signal is defined below. The function is then run with each
% element.
u_vec = NaN(1, N);
for i = 1:N
    if y_data(i) > 1.0
        u_vec(i) = 1.0;
    else
        u_vec(i) = 0.0;
    end
end

% Initializing the simulation results
mk = NaN(2, N, M_size);
mk(:, 1, :) = zeros(2, 1, M_size);

% Running simulation
<<<<<<< HEAD
<<<<<<< HEAD
for j = 1:M_size
    for i = 2:N
        mk(:, i, j) = disc_diff_eq(phi_first_guess, mk(:, i-1, j), u_vec(i-1), M(j));
    end
=======
for i = 2:N
<<<<<<< HEAD
    mk(:, i) = disc_diff_eq(phi_first_guess, mk(:, i-1), u_vec(i-1));
>>>>>>> 4f56570 (Changes in the new estimator, will try to finalize tomorrow)
=======
    for j = 1:1000
        mk(:, i) = disc_diff_eq(phi_first_guess, mk(:, i-1), u_vec(i-1),M(j));
=======
for j = 1:M_size
    for i = 2:N
        mk(:, i, j) = disc_diff_eq(phi_first_guess, mk(:, i-1, j), u_vec(i-1), M(j));
>>>>>>> e10bbb0 (New parameter estimation changes)
    end
>>>>>>> 6d69de0 (M - changes)
end

% Splitting the results

% % Plotting the results
% for i = 1:M_size
%     figure(1)
%     hold on
%     plot(t_vec, mk(1, :, i));
%     plot(t_vec, mk(2, :, i));
%     legend("Active Muscle Mass", "Fatigued Muscle Mass");
%     xlabel("Time (s)")
%     ylabel("Mass (kg)")
%     title("Hand Grip Model Simulation")
% end
% hold off

%% LEAST SQUARES ESTIMATION
% In order to find the theta-parameters, we need to declare them as
% optimization variables.
% The objective function is the sum of squares of the differences between
% the "real" solution and the data. In order to define the objective
% function, we need to import the function which computes the ODE with the
% parameters
type disc_theta_to_ode

<<<<<<< HEAD
<<<<<<< HEAD
phi = optimvar('phi', 4);

% Now, we express this function as an optimization expression.
optim_y = optimexpr(1, N);
sumsq = NaN(1, M_size);
phi_estims = NaN(4, M_size);

for i = 1:M_size
    fprintf("Problem %i", i);
    fcn = fcn2optimexpr(@disc_theta_to_ode, phi, N, u_vec, M(i));
    optim_y = fcn;
    % Finally, the objective function can be defined.
    obj = sum((y_data - optim_y).^2);
    % Now, the optimization problem
    opts = optimoptions(@fmincon, "MaxFunEvals", 9e3);
    prob = optimproblem("Objective", obj);
    prob.Constraints.cons1 = -phi(1) + phi(4) + phi(2) + (M(i)*phi(3)) >= 0.0001;
    prob.Constraints.cons2 = phi(1) + phi(4) - (2*phi(1)*phi(4)) + (phi(4)*phi(2)) + (M(i)*phi(4)*phi(3)) >= 1.0001;
    prob.Constraints.cons3 = -phi(1) + phi(4) + phi(3) + (M(i)*phi(3)) >= 0.0001;
    prob.Constraints.cons4 = phi(1) + phi(4) - (2*phi(1)*phi(4)) - (phi(1)*phi(3)) + (phi(4)*phi(3)) + (M(i)*phi(4)*phi(3)) >= 1.0001;
    % Initial guess on theta
    phi_0.phi = phi_first_guess;
    % Solve the optimization problem
    [phi_sol, sumsq(i)] = solve(prob, phi_0, 'Options', opts);
    phi_estims(:, i) = phi_sol.phi;
=======
phi = optimvar('phi', 5);
=======
phi = optimvar('phi', 4);
>>>>>>> 6d69de0 (M - changes)

% Now, we express this function as an optimization expression.
%fcnt = @(theta) theta_to_ode(theta, tspan, m0, u);
optim_y = optimexpr(1, N);
sumsq = NaN(1, M_size);
phi_estims = NaN(M_size, 4);

<<<<<<< HEAD
% Finally, the objective function can be defined.
obj = sum((y_data - optim_y).^2);

% Now, the optimization problem
prob = optimproblem("Objective", obj);
%% OPTIMIZATION PROBLEM: SOLVE
% Initial guess on theta
phi_0.phi = phi_first_guess;

% Solve the optimization problem
[phi_sol, sumsq] = solve(prob, phi_0);

disp(phi_sol.phi)
disp(sumsq)
%% PLOT ALL RESULTS
m_est = NaN(2, N);
m_est(:, 1) = zeros(2, 1);
for i = 2:N
<<<<<<< HEAD
<<<<<<< HEAD
    m_est(:, i) = disc_diff_eq(m_est(:, i-1), phi_sol.phi, u_vec(i));
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
=======
    m_est(:, i) = disc_diff_eq(phi_sol.phi, m_est(:, i-1), u_vec(i));
>>>>>>> 4f56570 (Changes in the new estimator, will try to finalize tomorrow)
=======
    m_est(:, i) = disc_diff_eq(phi_sol.phi, m_est(:, i-1), u_vec(i), M);
>>>>>>> 6d69de0 (M - changes)
=======
for i = 1:M_size
    fcn = fcn2optimexpr(@disc_theta_to_ode, phi, N, u_vec, M(i));
    optim_y = fcn(1, :);
    % Finally, the objective function can be defined.
    obj = sum((y_data - optim_y).^2);
    % Now, the optimization problem
    prob = optimproblem("Objective", obj);
    % Initial guess on theta
    phi_0.phi = phi_first_guess;
    % Solve the optimization problem
    [phi_sol, sumsq(i)] = solve(prob, phi_0);
    phi_estims(i, :) = phi_sol.phi;
>>>>>>> e10bbb0 (New parameter estimation changes)
end

[min, min_index] = min(sumsq);

%% PLOT ALL RESULTS
m_est = NaN(2, N, M_size);
m_est(:, 1, :) = zeros(2, 1, M_size);
for j = 1:M_size
    for i = 2:N
<<<<<<< HEAD
        m_est(:, i, j) = disc_diff_eq(phi_estims(:, j), m_est(:, i-1, j), u_vec(i), M(j));
=======
        m_est(:, i, j) = disc_diff_eq(phi_estims(j, :), m_est(:, i-1), u_vec(i), M(j));
>>>>>>> e10bbb0 (New parameter estimation changes)
    end
end

figure(1)
hold on
plot(t_vec, mk(1, :, min_index), '--');
plot(t_vec, mk(2, :, min_index), '--');
plot(t_vec, m_est(1, :, min_index));
plot(t_vec, m_est(2, :, min_index));
plot(t_vec, y_data);
plot(t_vec, u_vec);
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")
<<<<<<< HEAD
hold off

disp(phi_estims(:, min_index))
disp(M(min_index))
disp(sumsq(min_index))
=======
hold off
>>>>>>> e10bbb0 (New parameter estimation changes)
