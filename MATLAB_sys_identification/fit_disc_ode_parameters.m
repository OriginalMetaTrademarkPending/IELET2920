% We start by importing the discretized differential equations (via Forward
% Euler)
type disc_diff_eq

%% INITIALIZING SIMULATION
tspan = 60;           %Time span of the simulation in seconds
N = 590;              %Number of samples to be registered
t_vec = linspace(0, tspan, N);      %Time vector for plotting and input generation

% Defining the phi parameters. These parameters are defined as the theta
% parameters adjusted for the sample time. These parameters must be within
% 0 and 1. The last parameter is the total muscle mass. This parameter does
% not need to be adjusted for the sample time.
phi_first_guess = [0.05, 1, 0.3, 1.0, 7];

% The input signal is defined below. The function is then run with each
% element.
u = @(t) heaviside(t-10) - heaviside(t - 30) + heaviside(t - 40) - heaviside(t - 50);
u_vec = NaN(N, 1);
for i = 1:N
    u_vec(i) = u(t_vec(i));
end
% Initializing the simulation results
mk = NaN(N, 2);
mk(1, :) = zeros(1, 2);

% Running simulation
for i = 2:N
    mk(i, :) = disc_diff_eq(mk(i-1, :), phi_first_guess, u_vec(i));
end

% Splitting the results
m_active = mk(:, 1);
m_fatig = mk(:, 2);

% % Plotting the results
% plot(t_vec, m_active);
% hold on
% plot(t_vec, m_fatig);
% hold off
% legend("Active Muscle Mass", "Fatigued Muscle Mass"); %"Forward Euler Active Muscle Mass", "Forward Euler Fatigued Muscle Mass");
% xlabel("Time (s)")
% ylabel("Mass (kg)")
% title("Hand Grip Model Simulation")

%% LEAST SQUARES ESTIMATION
% Next, we import the data retrieved from the system testing, as well as
% the starting points. For this we need the filepath where the readings
% are.
FILEPATH = "/Users/admir/Desktop/BIELEKTRO/3. år/IELET2920 Bacheloroppgave automatisering/github-repo/IELET2920/python_scripts/data2.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data;

% In order to find the theta-parameters, we need to declare them as
% optimization variables.
phi = optimvar('phi', 5, 'LowerBound', [0, 0, 0, 0, 0], 'UpperBound',[1, 1, 1, 1, 10]);

% The objective function is the sum of squares of the differences between
% the "real" solution and the data. In order to define the objective
% function, we need to import the function which computes the ODE with the
% parameters
type disc_theta_to_ode

% Now, we express this function as an optimization expression.
%fcnt = @(theta) theta_to_ode(theta, tspan, m0, u);
fcn = fcn2optimexpr(@disc_theta_to_ode, phi, N, u_vec);
fcn_2_compare = fcn(:, 1);

% Finally, the objective function can be defined.
obj = sum((fcn_2_compare - y_data).^2);

% Now, the optimization problem
prob = optimproblem("Objective", obj);

% Initial guess on theta
phi_0.phi = phi_first_guess;

% Solve the optimization problem
[phi_sol, sumsq] = solve(prob, phi_0);

disp(phi_sol.phi)
disp(sumsq)
%% PLOT ALL RESULTS
m_est = NaN(N, 2);
m_est(1, :) = zeros(1, 2);
for i = 2:N
    m_est(i, :) = disc_diff_eq(m_est(i-1, :), phi_sol.phi, u_vec(i));
end
m_est_active = m_est(:, 1);
m_est_hidden = m_est(:, 2);

figure(2)
plot(t_vec, m_active, '--');
hold on
plot(t_vec, m_fatig, '--');
plot(t_vec, m_est_active);
plot(t_vec, m_est_hidden);
plot(t_vec, y_data)
hold off
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")