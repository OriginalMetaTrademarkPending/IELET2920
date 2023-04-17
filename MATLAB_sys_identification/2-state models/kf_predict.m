function [x_bar, P_bar] = kf_predict(x_hat, P_hat, phi, u, Q)
% Prediction step of the Kalman filter.
% The phi parameters are the best calculated parameters for the given
% optimization problem (They are thus required to be single/double).
    if u == 0
        F = [phi(1) - phi(2), 1 - phi(4);
            1 - phi(1), phi(4)];
    else
        F = [phi(1) - phi(3), 1 - phi(4) - phi(3);
            1 - phi(1), phi(4)];
    end
    B = [phi(3)*phi(5); 0];
    x_bar = (F*x_hat) + (B*u);
    P_bar = F*P_hat*F' + Q;
end