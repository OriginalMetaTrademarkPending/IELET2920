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

% Somewhere here, we need the actual parameters...the least squares problem
% requires the response from the system with real parameters (or as real 
% as they can get)...
% Until further notice, do some guesstimates, see how the system reacts
theta_real = [4, -3.9, 0.003, -4];
m0 = [0 0];
soltrue = ode45(@(t, m)diff_eq(t, m, theta_real, u(t)), tspan, m0);
m_true = deval(soltrue, tspan);
y_true = m_true(1, :);
y_hidden = m_true(2, :);
plot(tspan, y_true);
hold on
plot(tspan, y_hidden);
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

