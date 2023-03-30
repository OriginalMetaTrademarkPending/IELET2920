function mkp1 = disc_diff_eq(mk, phi, h, u)
% The differential equations describing hand grip dynamics, discretized
% through forward Euler
%   mk -> vector consisting of active muscle mass m_a and fatigued muscle
%   mass m_f at sample k. m_a is the first element and m_f is the 
%   second element.
%   phi -> vector consisting of the discrete parameters to be evaluated.
%   phi(1) = theta_af
%   phi(2) = theta_ar
%   phi(3) = theta_ra
%   phi(4) = theta_fa
%   phi(5) = M
%   u -> scalar which serves as the control input for the model.
mkp1 = [(1 - (h*(phi(1) + phi(2) - u*(phi(2) - phi(3)))*mk(1))) + (h*(phi(4) - (u*phi(3))))*mk(2) + (h*phi(3)*phi(5)*u);
    (h*phi(1))*mk(1) + (1 - (h*phi(4))*mk(2))];
end

