function dmdt = diff_eq_cramp(~, m, theta, u)
% The differential equations describing hand grip dynamics
%   m -> vector consisting of active muscle mass m_a, fatigued muscle
%   mass m_f and cramped muscle mass m_c. m_a is the first element, m_f is 
%   the second element and m_c is the third element.
%   theta -> vector consisting of the parameters to be evaluated.
%   theta(1) = theta_af
%   theta(2) = theta_ar
%   theta(3) = theta_ra
%   theta(4) = theta_fa
%   theta(5) = M
%   theta(6) = theta_ac
%   theta(7) = theta_cr
%   u -> scalar which serves as the control input for the model.
dmdt = [((-theta(1)-theta(2)-theta(6) + ((theta(2) - theta(3))*u))*m(1)) + ((theta(4) - (theta(3)*u))*m(2)) - (theta(3)*u*m(3)) + (theta(3)*theta(5)*u);
    (theta(1)*m(1)) - (theta(4)*m(2));
    (theta(6)*m(1)) - (theta(7)*m(3))];
end
