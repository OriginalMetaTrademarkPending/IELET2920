function mkp1 = disc_diff_eq(mk, theta, h, u)
% The differential equations describing hand grip dynamics, discretized
% through forward Euler
%   mk -> vector consisting of active muscle mass m_a and fatigued muscle
%   mass m_f at sample k. m_a is the first element and m_f is the 
%   second element.
%   phi -> vector consisting of the discrete parameters to be evaluated.
%   theta(1) = theta_af
%   theta(2) = theta_ar
%   theta(3) = theta_ra
%   theta(4) = theta_fa
%   theta(5) = M
%   u -> scalar which serves as the control input for the model.
%   The theta parameters are getting changed into phi parameters. These phi
%   parameters are getting constrained between 0 and 1.
phi = [1 - (h*(theta(1) + theta(2) - u*(theta(2) - theta(3))));
       h*(theta(4) - (u*theta(3)));
       h*theta(3)*theta(5);
       h*theta(1);
       1-(h*theta(4))];
mkp1 = [(phi(1)*mk(1)) + (phi(2)*mk(2)) + (phi(3)*u); (phi(4)*mk(1)) + (phi(5)*mk(2))];
end

