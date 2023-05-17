<<<<<<< HEAD
function [x_hat, P_hat, z] = kf_update(y, x_bar, P_bar, H, R)
=======
function [x_hat, P_hat] = kf_update(y, x_bar, P_bar, H, R)
>>>>>>> 9f9fe91 (Implemented the first part of the estimator (Kalman Filter). Trying to find a solution for the least squares estimator)
% Update step of the Kalman filter
% First, calculate the residual
z = y - H*x_bar;

% Then, calculate the innovation matrix
S = H*P_bar*H' + R;

% Calculate the Kalman gain for the filter
<<<<<<< HEAD
K = P_bar*H'/S;
=======
K = P_bar*H'*inv(S);
>>>>>>> 9f9fe91 (Implemented the first part of the estimator (Kalman Filter). Trying to find a solution for the least squares estimator)

% Calculate the estimates and the state covariance matrix
x_hat = x_bar + (K*z);
P_hat = (eye(2) - (K*H))*P_bar*(eye(2) - (K*H))' + (K*R*K');     %Using the Joseph equation for the state covariance.
end

