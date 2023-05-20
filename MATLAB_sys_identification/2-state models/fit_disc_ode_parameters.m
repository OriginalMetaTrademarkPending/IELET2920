% We start by importing the discretized differential equations (via Forward
% Euler)
clear
type disc_diff_eq

% Next, we import the data retrieved from the system testing, as well as
% the starting points. For this we need the filepath where the readings
% are.
FILEPATH = "../../python_scripts/test_bias8.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data = readings.Var1';
N = max(size(y_data));      %Number of samples to be registered
tspan = 360;                %Time span of the simulation in seconds
M_size = 100;

% % some data cleanup. Comment away if no noise is found
% for i = 1:N
%     if(i > 9381)
%         y_data(i) = 0.0;
%     end
% end
%% INITIALIZING SIMULATION
t_vec = linspace(0, tspan, N); %Time vector for plotting and input generation

% Defining the phi parameters. These parameters are defined as the theta
% parameters adjusted for the sample time. These parameters must be within
% 0 and 1. The last parameter is the total muscle mass. This parameter does
% not need to be adjusted for the sample time, but will be included as a
% family of different parameters.
phi_first_guess = [0.5, 0.5, 0.5, 0.5]; 
M = linspace(3, 40, M_size);

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
for j = 1:M_size
    for i = 2:N
        mk(:, i, j) = disc_diff_eq(phi_first_guess, mk(:, i-1, j), u_vec(i-1), M(j));
    end
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
end

[min, min_index] = min(sumsq);

%% PLOT ALL RESULTS
m_est = NaN(2, N, M_size);
m_est(:, 1, :) = zeros(2, 1, M_size);
for j = 1:M_size
    for i = 2:N
        m_est(:, i, j) = disc_diff_eq(phi_estims(:, j), m_est(:, i-1, j), u_vec(i), M(j));
    end
end

figure(1)
hold on
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
