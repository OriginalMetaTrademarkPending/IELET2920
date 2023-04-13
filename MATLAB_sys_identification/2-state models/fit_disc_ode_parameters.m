% We start by importing the discretized differential equations (via Forward
% Euler)
type disc_diff_eq

% Next, we import the data retrieved from the system testing, as well as
% the starting points. For this we need the filepath where the readings
% are.
FILEPATH = "../../python_scripts/test2.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Data1;
N = max(size(y_data));      %Number of samples to be registered
tspan = 60;                %Time span of the simulation in seconds
%% INITIALIZING SIMULATION
t_vec = linspace(0, tspan, N);      %Time vector for plotting and input generation

% Defining the phi parameters. These parameters are defined as the theta
% parameters adjusted for the sample time. These parameters must be within
% 0 and 1. The last parameter is the total muscle mass. This parameter does
% not need to be adjusted for the sample time.
phi_first_guess = [0.3, 0.9, 0.9, 0.8, 10];

% The input signal is defined below. The function is then run with each
% element.
u_vec = NaN(N, 1);
for i = 1:N
    if y_data(i) > 1.0
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
    mk(:, i) = disc_diff_eq(mk(:, i-1), phi_first_guess, u_vec(i));
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

%% LEAST SQUARES ESTIMATION
% In order to find the theta-parameters, we need to declare them as
% optimization variables.
phi = optimvar('phi', 5, 'LowerBound', [0, 0, 0, 0, 0], 'UpperBound', [1, 1, 1, 1, 20]);

% The objective function is the sum of squares of the differences between
% the "real" solution and the data. In order to define the objective
% function, we need to import the function which computes the ODE with the
% parameters
type disc_theta_to_ode

% Now, we express this function as an optimization expression.
%fcnt = @(theta) theta_to_ode(theta, tspan, m0, u);
fcn = fcn2optimexpr(@disc_theta_to_ode, phi, N, u_vec);

% Finally, the objective function can be defined.
obj = sum((fcn - y_data').^2);

% Now, the optimization problem
prob = optimproblem("Objective", obj);

% %% OPTIMIZATION PROBLEM: CONSTRAINTS
% % We find the constraints by performing eigenvalue decomposition on the
% % matrices we get by setting u = 0 and u = 1. First, define these matrices
% % through optimization variables.
% A_0 = [phi(1) - phi(2), 1-phi(4);
%     1 - phi(1), phi(4)];
% 
% A_1 = [phi(1) - phi(3), 1 - phi(4) - phi(3);
%     1 - phi(1), phi(4)];
% % The eigenvalues above are constrained within 0 and 1 (for a stable system
% % in general). This yields 8 constraints.
% prob.Constraints.cons1 = min(eigs(A_0)) >= 0;
% prob.Constraints.cons2 = max(eigs(A_0)) <= 1;
% prob.Constraints.cons3 = min(eigs(A_1)) >= 0;
% prob.Constraints.cons4 = max(eigs(A_1)) <= 1;

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
plot(t_vec, y_data);
plot(t_vec, u_vec);
hold off
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")