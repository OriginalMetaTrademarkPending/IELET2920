function mkp1 = disc_diff_eq(mk, phi, u)
% The differential equations describing hand grip dynamics, discretized
% through forward Euler
%   mk -> vector consisting of active muscle mass m_a and fatigued muscle
%   mass m_f at sample k. m_a is the first element and m_f is the 
%   second element.
%   phi -> vector consisting of the discrete parameters to be evaluated.
%   phi(1) = 1 - h(theta_af) = phi_af
%   phi(2) = h(theta_ar) = phi_ar
%   phi(3) = h(theta_ra) = phi_ra
%   phi(4) = 1 - h(theta_fa) = phi_fa
%   phi(5) = M
%   u -> scalar which serves as the control input for the model.
p_af = phi(1); p_ar = phi(2); p_ra = phi(3); p_fa = phi(4); M = phi(5);

A = [p_af - p_ar + (p_ar - p_ra)*u, 1 - p_fa - (p_ra*u);
     1 - p_af, p_fa];

B = [p_ra*M; 0];

mkp1 = A*mk + B*u;
end

