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

%% EXTENDED ESTIMATOR IMPLEMENTATION
% Declaring some constants. Here, the the measurement
% function is a constant.
H = [1 0];
% In order to implement the Kalman filter, we first need to declare the
% initial state estimate and the initial state covariance. We also need to
% preallocate the predicted state and prediction covariance, as well as the
% residual vector
x_hat(:, 1) = zeros(2, 1);
P_hat = zeros(2, 2);

x_bar = NaN(2, N);
P_bar = zeros(2, 2);

z = NaN(1, N);

% Initializing the "optimal" parameters.
phi_star.phi = phi_0;
phi_star1.phi = phi_0;
phi_star2.phi = phi_0;

% Initializing the optimization variables for the least squares estimation
optim_phi = optimvar('phi', 5, 'LowerBound', [0, 0, 0, 0, 0]);

% Values for checking whether the algorithm works or not
phi_evo = NaN(5, N);
sumsq_evo = NaN(2, N);

% Calling in the iterations
for i = 2:N
    [x_bar(:, i-1), P_bar] = kf_predict(x_hat(:, i-1), P_hat, phi_star.phi, u_vec(i-1), Q);
    [x_hat(:, i), P_hat, z(i-1)] = kf_update(y(i), x_bar(:, i-1), P_bar, H, R);
    %%% WILL WRITE THE ESTIMATOR CODE TOMORROW! %%%
end
%% PLOT ALL RESULTS
figure(1)
hold on
plot(t_vec(1:i), x_hat(1, :));
plot(t_vec(1:i), y(1:i));
plot(t_vec(1:i), u_vec(1:i));
hold off
legend("Estimated Active Mass", "Measurement", "Input");
xlabel("Time (s)")
ylabel("Mass (kg)")
title("Hand Grip System Identification")

%figure(2)
%plot(t_vec, z);
