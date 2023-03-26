% We start by importing the differential equations which describe the
% system.
type diff_eq

%% TESTING THE DYNAMICS
% The experiment shall last for 30 seconds. This is our timespan. In
% addition, measurements will be taken every 1 ms (corresponds to 30000
% samples/data points).
tspan = 0:0.001:30;

% The input signal is defined below as a square wave with on-time of 10 
% seconds and off-time of 5 seconds.
u = @(t) heaviside(t - 5) - heaviside(t - 15) + heaviside(t - 20) - heaviside(t - 30);
u_1 = @(t) heaviside(t);

% Somewhere here, we need the actual parameters...the least squares problem
% requires the response from the system with real parameters (or as real 
% as they can get)...
% Until further notice, do some guesstimates, see how the system reacts
% guesstimate 1: theta(1) = 2, theta(2) = -1000, theta(3) = 2000, theta(4)
% = -9
% guesstimate 2: theta(1) = 0.2, theta(2) = -100, theta(3) = 200, theta(4)
% = -2
% guesstimate 3: theta(1) = 0.025, theta(2) = 0.05, theta(3) = 1, theta(4)
% = -0.4

% GUESSTIMATE RULES
% 1. The sum of theta(1) and theta(2) must not be larger than 1 (the lower,
% the better).
% 2. Theta(4) must be larger than -1.
% 3. Theta(3) must be approximately 1.
% 4. Theta(3) must be larger than theta(1).
theta_real = [0.025, 0.05, 1, -0.4];
m0 = [0 0];
soltrue = ode45(@(t, m)diff_eq(t, m, theta_real, u(t)), tspan, m0);
soltrue_1 = ode45(@(t, m)diff_eq(t, m, theta_real, u_1(t)), tspan, m0);
m_true = deval(soltrue, tspan);
m_true_1 = deval(soltrue_1, tspan);
y_true = m_true(1, :);
y_hidden = m_true(2, :);
y_true_1 = m_true_1(1, :);
y_hidden_1 = m_true_1(2, :);

figure(1)
plot(tspan, y_true);
hold on
plot(tspan, y_hidden);
hold off

figure(2)
plot(tspan, y_true_1);
hold on
plot(tspan, y_hidden_1);
hold off

%%
% Next, we import the data retrieved from the system testing, as well as
% the starting points.
m_data = 0;
m0 = zeros(30000, 1);

% In order to find the theta-parameters, we need to declare them as
% optimization variables.
theta = optimvar('theta', 5);

% The objective function is the sum of squares of the differences between
% the "real" solution and the data. In order to define the objective
% function, we need to import the function which computes the ODE with the
% parameters
type theta_to_ode

% Now, we express this function as an optimization expression.
fcn = fcn2optimexpr(@theta_to_ode, theta, tspan, m0);

% Finally, the objective function can be defined.
obj = sum(sum((y_data - y_true).^2));

% Now, the optimalization problem
prob = optimproblem("Objective", obj);

