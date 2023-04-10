% We start by importing the differential equations which describe the
% system.
type diff_eq
%% DATA ACQUISITION AND INPUT ESTIMATION
% Next, we import the data retrieved from the system testing, as well as
% the starting points. For this we need the filepath where the readings
% are.
FILEPATH = "../../python_scripts/2-sensor-calib-test.csv";
readings = readtable(FILEPATH, 'VariableNamingRule', 'preserve');
y_data_1 = readings.Data1;
y_data_2 = readings.Data2;

% Setting up the simulation time and the number of samples based on the
% data we received
N = max(size(y_data_1));
tspan = linspace(0, 120, N);

% This code block estimates the input of the system based on the
% measurements.
u1 = zeros(N, 1);
u2 = zeros(N, 1);
for i = 1:N
    if y_data_1(i) > 1.0
        u1(i) = 1.0;
    else
        u1(i) = 0.0;
    end
    if y_data_2(i) > 1.0
        u2(i) = 1.0;
    else
        u2(i) = 0.0;
    end
end

%% SIMULATION OF THE SYSTEM (WITH INITIAL GUESSES)
% The experiment shall last for 30 seconds. This is our timespan. In
% addition, measurements will be taken every 1 ms (corresponds to 30000
% samples/data points).
% The input signal is defined below
% u = @(t) heaviside(t) - heaviside(t - 8.8) + heaviside(t - 27.5) - heaviside(t - 47.7);
% u = 0.5*square((2*pi*tspan./40) + (pi/2)) + 0.5;
% u = 1;
%u_1 = @(t) heaviside(t);

% Somewhere here, we need the actual parameters...the least squares problem
% requires the response from the system with real parameters (or as real 
% as they can get)...
% Until further notice, do some guesstimates, see how the system reacts
% guesstimate 1: theta(1) = 2, theta(2) = -1000, theta(3) = 2000, theta(4)
% = -9
% guesstimate 2: theta(1) = 0.2, theta(2) = -100, theta(3) = 200, theta(4)
% = -2
% guesstimate 3: theta(1) = 0.025, theta(2) = 0.05, theta(3) = 1, theta(4)
% = 0.4
% guesstimate 4: theta(1) = 0.1394, theta(2) = 0.1766, theta(3) = 1.3288,
% theta(4) = 0.1021  (generated by 1 iteration of leastsq)

% GUESSTIMATE RULES OF THUMB
% 1. The sum of theta(1) and theta(2) must not be larger than 1 (the lower,
% the better).
% 2. Theta(4) must be between 1 and 0.
% 3. Theta(3) must be approximately 1.
% 4. Theta(3) must be larger than theta(1).
theta_real = [0.1394 0.1766 1.3288 0.1021 8];
m0 = [0 0];
% soltrue = ode45(@(t, m)diff_eq(t, m, theta_real, u(t)), tspan, m0);
% m_true = deval(soltrue, tspan);
% y_true = m_true(1, :);
% y_hidden = m_true(2, :);

% New solution. Solve via ode45 for each sample, then save the data in the
% different vectors.
sol_timeframe_1 = NaN(2, N);
sol_timeframe_1(:, 1) = m0';
sol_timeframe_2 = NaN(2, N);
sol_timeframe_2(:, 1) = m0';
for i = 2:N
    part_sol_1 = ode45(@(t,m)diff_eq(t, m, theta_real, u1(i)), [tspan(i-1), tspan(i)], sol_timeframe_1(:, i-1));
    part_sol_2 = ode45(@(t,m)diff_eq(t, m, theta_real, u2(i)), [tspan(i-1), tspan(i)], sol_timeframe_2(:, i-1));
    sol_timeframe_1(:, i) = part_sol_1.y(:, end);
    sol_timeframe_2(:, i) = part_sol_2.y(:, end);
end

y_true_1 = sol_timeframe_1(1, :);
y_hidden_1 = sol_timeframe_1(2, :);
y_true_2 = sol_timeframe_2(1, :);
y_hidden_2 = sol_timeframe_2(2, :);

%% LEAST SQUARES ESTIMATOR
% In order to find the theta-parameters, we need to declare them as
% optimization variables.
theta_index = optimvar('theta_i', 5, 'LowerBound', [0, 0, 0, 0, 0]);
theta_middle = optimvar('theta_m', 5, 'LowerBound', [0, 0, 0, 0, 0]);

% The objective function is the sum of squares of the differences between
% the "real" solution and the data. In order to define the objective
% function, we need to import the function which computes the ODE with the
% parameters
type theta_to_ode

% Now, we express this function as an optimization expression.
%fcnt = @(theta) theta_to_ode(theta, tspan, m0, u);
fcn1 = fcn2optimexpr(@theta_to_ode, theta_index, tspan, m0, u1, N);
fcn2 = fcn2optimexpr(@theta_to_ode, theta_middle, tspan, m0, u2, N);
%show(fcn)

% Finally, the objective function can be defined.
obj1 = sum((fcn1 - y_data_1).^2);
obj2 = sum((fcn2 - y_data_2).^2);

% Now, the optimization problem
prob_i = optimproblem("Objective", obj1);
prob_m = optimproblem("Objective", obj2);

% Initial guess on theta
theta_0_i.theta_i = theta_real;
theta_0_m.theta_m = theta_real;

% Solve the optimization problem
[theta_sol_i, sumsq_i] = solve(prob_i, theta_0_i);
[theta_sol_m, sumsq_m] = solve(prob_m, theta_0_m);

disp(theta_sol_i.theta_i)
disp(sumsq_i)

disp(theta_sol_m.theta_m)
disp(sumsq_m)

%% PLOT ALL RESULTS
solest = NaN(2, N);
solest(:, 1) = m0;
for i = 2:N
    part_solest = ode45(@(t,m)diff_eq(t, m, theta_sol_i.theta, u1(i)), [tspan(i-1), tspan(i)], solest(:, i-1));
    solest(:, i) = part_solest.y(:, end);
end
y_est_active = solest(1, :);
y_est_hidden = solest(2, :);

figure(1)
plot(tspan, y_true, '--');
hold on
plot(tspan, y_hidden, '--');
plot(tspan, y_est_active);
plot(tspan, y_est_hidden);
plot(tspan, y_data_1)
plot(tspan, u1);
hold off
legend("Active Muscle Mass", "Fatigued Muscle Mass", "Estimated Active Muscle Mass", "Estimated Fatigued Muscle Mass", "Input");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")