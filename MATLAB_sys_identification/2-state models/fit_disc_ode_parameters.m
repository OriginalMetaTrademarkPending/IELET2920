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
=======
FILEPATH = "../../python_scripts/test_movavg_10ms.csv";
>>>>>>> a7ffe45 (New changes to parameter estimation)
=======
FILEPATH = "../../python_scripts/test_movavg_10ms5.csv";
>>>>>>> 9b8dcd8 (bigger movavg)
=======
FILEPATH = "../../python_scripts/test_movavg_10ms6.csv";
>>>>>>> 6ff7809 (functions for arduino code)
=======
FILEPATH = "../../python_scripts/test1.csv";
>>>>>>> a3698cc (Last changes to the files)
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data1';
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
N = max(size(y_data));      %Number of samples to be registered
<<<<<<< HEAD
tspan = 180;                %Time span of the simulation in seconds
=======
FILEPATH = "../../python_scripts/test_bias2.csv";
=======
FILEPATH = "../../python_scripts/Test_movavg_10ms6.csv";
>>>>>>> 4eaf0d9 (Plots for the report, as well as a new constraint calculation)
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data1';
N = max(size(y_data));      %Number of samples to be registered
<<<<<<< HEAD
tspan = 60;                %Time span of the simulation in seconds
>>>>>>> dc87003 (test bias1 and 2)
=======
tspan = 300;                 %Time span of the simulation in seconds
>>>>>>> 4eaf0d9 (Plots for the report, as well as a new constraint calculation)
M_size = 100;
=======
tspan = 240;                %Time span of the simulation in seconds
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 564e173 (Massive changes to code, and perhaps a new success???)
=======
M_size = 50;
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
M_size = 100;
>>>>>>> a7ffe45 (New changes to parameter estimation)
=======
FILEPATH = "../../python_scripts/Test_movavg_10ms6.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data1';
N = max(size(y_data));      %Number of samples to be registered
=======
FILEPATH = "../../python_scripts/test_bias8.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Var1';
N = max(size(y_data));      %Number of samples to be registered
>>>>>>> master
tspan = 360;                %Time span of the simulation in seconds
M_size = 100;

% % some data cleanup. Comment away if no noise is found
% for i = 1:N
%     if(i > 9381)
%         y_data(i) = 0.0;
%     end
% end
<<<<<<< HEAD
>>>>>>> 2393880 (Final updates to the code)
=======
>>>>>>> master
%% INITIALIZING SIMULATION
t_vec = linspace(0, tspan, N); %Time vector for plotting and input generation

% Defining the phi parameters. These parameters are defined as the theta
% parameters adjusted for the sample time. These parameters must be within
% 0 and 1. The last parameter is the total muscle mass. This parameter does
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
% not need to be adjusted for the sample time, but will be included as a
% family of different parameters.
phi_first_guess = [0.9, 0.1, 0.1, 0.9]; 
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
<<<<<<< HEAD
phi_first_guess = [0.2, 0.5, 0.9, 0.7]; 
<<<<<<< HEAD
M = linspace(3, 30, 100);
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
M = linspace(3, 50, 1000);
>>>>>>> cd85389 (git playing games)
% af, 
>>>>>>> 61698af (Circuit and PCB)
=======
phi_first_guess = [0.5, 0.5, 0.5, 0.5]; 
M = linspace(3, 40, M_size);
>>>>>>> a7ffe45 (New changes to parameter estimation)
=======
% not need to be adjusted for the sample time, but will be included as a
% family of different parameters.
phi_first_guess = [0.5, 0.5, 0.5, 0.5]; 
M = linspace(3, 40, M_size);
>>>>>>> master

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
<<<<<<< HEAD
=======
>>>>>>> master
for j = 1:M_size
    for i = 2:N
        mk(:, i, j) = disc_diff_eq(phi_first_guess, mk(:, i-1, j), u_vec(i-1), M(j));
    end
<<<<<<< HEAD
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

=======
end

% Splitting the results

>>>>>>> master
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
phi = optimvar('phi', 4);
=======
phi = optimvar('phi', 4, 'LowerBound', [0, 0, 0, 0], 'UpperBound', [1, 1, 1, 1]);
>>>>>>> a7ffe45 (New changes to parameter estimation)

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
=======
phi = optimvar('phi', 4, 'LowerBound', [0, 0, 0, 0]);

% Now, we express this function as an optimization expression.
>>>>>>> cd85389 (git playing games)
optim_y = optimexpr(1, N);
sumsq = NaN(1, M_size);
phi_estims = NaN(4, M_size);

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
    prob.Constraints.cons1 = phi(4) + phi(2) - phi(1) + (M(i)*phi(3)) >= 0.00001;
    prob.Constraints.cons2 = (M(i)*phi(3)*phi(4)) + phi(1) + phi(4) - (2*phi(1)*phi(4)) + (phi(4)*phi(2)) >= 1.00001;
    prob.Constraints.cons3 = phi(3) + phi(4) - phi(1) + (M(i)*phi(3)) >= 0.00001;
    prob.Constraints.cons4 = (M(i)*phi(3)*phi(4)) + phi(1) + phi(4) + phi(3) - (2*phi(1)*phi(4)) - (phi(1)*phi(3)) + (phi(4)*phi(3)) >= 1.00001;
    % Initial guess on theta
    phi_0.phi = phi_first_guess;
    % Solve the optimization problem
    [phi_sol, sumsq(i)] = solve(prob, phi_0);
<<<<<<< HEAD
    phi_estims(i, :) = phi_sol.phi;
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
    phi_estims(:, i) = phi_sol.phi;
>>>>>>> cd85389 (git playing games)
=======
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
>>>>>>> master
end

[min, min_index] = min(sumsq);

%% PLOT ALL RESULTS
m_est = NaN(2, N, M_size);
m_est(:, 1, :) = zeros(2, 1, M_size);
for j = 1:M_size
    for i = 2:N
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        m_est(:, i, j) = disc_diff_eq(phi_estims(:, j), m_est(:, i-1, j), u_vec(i), M(j));
=======
        m_est(:, i, j) = disc_diff_eq(phi_estims(j, :), m_est(:, i-1), u_vec(i), M(j));
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
        m_est(:, i, j) = disc_diff_eq(phi_estims(:, j), m_est(:, i-1, j), u_vec(i), M(j));
>>>>>>> cd85389 (git playing games)
=======
        m_est(:, i, j) = disc_diff_eq(phi_estims(:, j), m_est(:, i-1, j), u_vec(i), M(j));
>>>>>>> master
    end
end

figure(1)
hold on
<<<<<<< HEAD
plot(t_vec, mk(1, :, min_index), '--');
plot(t_vec, mk(2, :, min_index), '--');
plot(t_vec, m_est(1, :, min_index));
plot(t_vec, m_est(2, :, min_index));
plot(t_vec, y_data);
plot(t_vec, u_vec);
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass");
<<<<<<< HEAD
<<<<<<< HEAD
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")
<<<<<<< HEAD
<<<<<<< HEAD
=======
xlabel("Time (s)", "FontSize", 14)
ylabel("Mass (kg)", "FontSize", 14)
title("Hand Grip System Identification: Test 12m", "fontSize", 22)
>>>>>>> 2393880 (Final updates to the code)
=======
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification: Test 12", "fontSize", 34)
>>>>>>> 39930d4 (Last commit?)
hold off
disp(phi_estims(:, min_index))
<<<<<<< HEAD
<<<<<<< HEAD
disp(M(min_index))
disp(sumsq(min_index))
=======
hold off
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
hold off

disp(phi_estims(:, min_index))
>>>>>>> cd85389 (git playing games)
=======
disp(M(min_index))
>>>>>>> a3698cc (Last changes to the files)
=======
disp(M(min_index))
disp(min);
>>>>>>> 4eaf0d9 (Plots for the report, as well as a new constraint calculation)
=======
plot(t_vec, mk(1, :, min_index), '--','LineWidth',7);
plot(t_vec, mk(2, :, min_index), '--','LineWidth',7);
plot(t_vec, m_est(1, :, min_index),'LineWidth',7);
plot(t_vec, m_est(2, :, min_index),'LineWidth',7);
plot(t_vec, y_data,'LineWidth',7);
plot(t_vec, u_vec,'LineWidth',7);
set(gca,"FontSize",50)
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass", "Data", "Input");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification: Test bias 8", "fontSize", 62)
hold off
disp(phi_estims(:, min_index))
disp(M(min_index))
disp(sumsq(min_index))
>>>>>>> master
