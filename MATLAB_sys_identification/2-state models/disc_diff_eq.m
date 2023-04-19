<<<<<<< HEAD
<<<<<<< HEAD
function mkp1 = disc_diff_eq(phi, mk, u, M)
=======
function mkp1 = disc_diff_eq(phi, mk, u)
>>>>>>> 9f9fe91 (Implemented the first part of the estimator (Kalman Filter). Trying to find a solution for the least squares estimator)
=======
function mkp1 = disc_diff_eq(phi, mk, u, M)
>>>>>>> 6d69de0 (M - changes)
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
<<<<<<< HEAD
p_af = phi(1); p_ar = phi(2); p_ra = phi(3); p_fa = phi(4);
=======
p_af = phi(1); p_ar = phi(2); p_ra = phi(3); p_fa = phi(4); 
>>>>>>> 6d69de0 (M - changes)

A = [p_af - p_ar*(1-u) - p_ra*u, 1 - p_fa - p_ra*u;
     1 - p_af, p_fa];

B = [p_ra*M; 0];

mkp1 = A*mk + B*u;

end

