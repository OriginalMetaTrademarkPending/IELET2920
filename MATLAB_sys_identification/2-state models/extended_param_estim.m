%% FINDING R AND TUNING Q
% In order to find R, we import the Estimate_R.csv file.
r_FILEPATH = "../../python_scripts/Estimate_R.csv";
r_readings = readtable(r_FILEPATH, 'VariableNamingRule', 'preserve');
r_data = r_readings.Data1';
R = cov(r_data);

% The Q vector is only available as a tuning variable. We have chosen to
% start with Q = 1/5*R in the diagonal, as well as 0 in the covariance
% elements.
Q = 0.2*R*eye(2);

%% MODEL IMPORT
% We start by importing the discretized differential equations (via Forward
% Euler)
type disc_diff_eq

% Next, we import the data retrieved from the system testing, as well as
% the starting points. For this we need the filepath where the readings
% are.
FILEPATH = "../../python_scripts/test6.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y = readings.Data1';
N = max(size(y));           %Number of samples to be registered
tspan = 240;                %Time span of the simulation in seconds
%% INITIALIZING SIMULATION
t_vec = linspace(0, tspan, N);      %Time vector for plotting and input generation

% Defining the phi parameters. These parameters are defined as the theta
% parameters adjusted for the sample time. These parameters must be within
% 0 and 1. The last parameter is the total muscle mass. This parameter does
% not need to be adjusted for the sample time.
phi_0 = [0.99, 0.7, 0.6, 0.9, 20]; 
% af, 

% The input signal is defined below. The function is then run with each
% element.
u_vec = NaN(1, N);
for i = 1:N
    if y(i) > 1.0
        u_vec(i) = 1.0;
    else
        u_vec(i) = 0.0;
    end
end

% Initializing the simulation results
mk = NaN(2, N);
mk(:, 1) = zeros(2, 1);

% Running simulation
for i = 2:N
    mk(:, i) = disc_diff_eq(phi_0, mk(:, i-1), u_vec(i-1));
end

% Splitting the results
m_active = mk(1, :);
m_fatig = mk(2, :);

% % Plotting the results
% plot(t_vec, m_active);
% hold on
% plot(t_vec, m_fatig);
% hold off
% legend("Active Muscle Mass", "Fatigued Muscle Mass"); %"Forward Euler Active Muscle Mass", "Forward Euler Fatigued Muscle Mass");
% xlabel("Time (s)")
% ylabel("Mass (kg)")
% title("Hand Grip Model Simulation")

%% KALMAN FILTER IMPLEMENTATION
% Declaring some constants. Here, the the measurement
% function is a constant.
H = [1 0];
% In order to implement the Kalman filter, we first need to declare the
% initial state estimate and the initial state covariance. We also need to
% preallocate the predicted state and prediction covariance.

x_hat = NaN(2, N);
x_hat(:, 1) = zeros(2, 1);
P_hat = zeros(2, 2);

x_bar = NaN(2, N);
P_bar = zeros(2, 2);

% Initializing the "optimal" parameters.
phi_star.phi = phi_0;

% Initializing the optimization variables for the least squares estimation
optim_phi = optimvar('phi', 5, 'LowerBound', [0, 0, 0, 0, 0]);

% Initializing the optimalization expression variables
optim_expr = optimexpr(2, N);

% Values for checking whether the algorithm works or not
phi_evo = NaN(5, N);
sumsq_evo = NaN(1, N);

% Calling in the iterations
for i = 1:N-1
    [x_bar(:, i), P_bar] = kf_predict(x_hat(:, i), P_hat, phi_star.phi, u_vec(i), Q);
    [x_hat(:, i+1), P_hat] = kf_update(y(i), x_bar(:, i), P_bar, H, R);
    % Declare the optimization function
    optim_expr(:, i) = fcn2optimexpr(@optimexpression, optim_phi, x_hat(:, i+1), u_vec(i));
    for j = 1:i
        obj = sum((optim_expr(:, j) - x_hat(:, j)).^2);
    end
    prob = optimproblem("Objective", obj);
    [phi_star, sumsq] = solve(prob, phi_star);
    phi_evo(:, i) = phi_star.phi;
    sumsq_evo(i) = sumsq;
end

disp(phi_evo)
disp(sumsq_evo)

%% LEAST SQUARES ESTIMATION
% In order to find the theta-parameters, we need to declare them as
% optimization variables.
% The objective function is the sum of squares of the differences between
% the "real" solution and the data. In order to define the objective
% function, we need to import the function which computes the ODE with the
% parameters
type disc_theta_to_ode

phi = optimvar('phi', 5);

% Now, we express this function as an optimization expression.
%fcnt = @(theta) theta_to_ode(theta, tspan, m0, u);
fcn = fcn2optimexpr(@disc_theta_to_ode, phi, N, u_vec);
optim_y = fcn(1, :);

% Finally, the objective function can be defined.
obj = sum((y - optim_y).^2);

% Now, the optimization problem
prob = optimproblem("Objective", obj);
%% OPTIMIZATION PROBLEM: SOLVE
% Initial guess on theta
phi_opt.phi = phi_0;

% Solve the optimization problem
[phi_sol, sumsq] = solve(prob, phi_opt);

disp(phi_sol.phi)
disp(sumsq)
%% PLOT ALL RESULTS
m_est = NaN(2, N);
m_est(:, 1) = zeros(2, 1);
for i = 2:N
    m_est(:, i) = disc_diff_eq(m_est(:, i-1), phi_sol.phi, u_vec(i));
end
m_est_active = m_est(1, :);
m_est_hidden = m_est(2, :);

figure(2)
plot(t_vec, m_active, '--');
hold on
plot(t_vec, m_fatig, '--');
plot(t_vec, m_est_active);
plot(t_vec, m_est_hidden);
plot(t_vec, y);
plot(t_vec, u_vec);
hold off
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")