function [x_hat, P_hat, z] = kf_update(y, x_bar, P_bar, H, R)
% Update step of the Kalman filter
% First, calculate the residual
z = y - H*x_bar;

% Then, calculate the innovation matrix
S = H*P_bar*H' + R;

% Calculate the Kalman gain for the filter
K = P_bar*H'/S;

% Calculate the estimates and the state covariance matrix
x_hat = x_bar + (K*z);
P_hat = (eye(2) - (K*H))*P_bar*(eye(2) - (K*H))' + (K*R*K');     %Using the Joseph equation for the state covariance.
end

