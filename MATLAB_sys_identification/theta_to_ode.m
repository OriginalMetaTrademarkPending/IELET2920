function solution = theta_to_ode(theta, tspan, m0, u)
%The parametric differential equation to be implemented in the objective
%function. Note that only the active muscle mass is measured, the fatigued
%muscle mass is a hidden variable.
sol = ode45(@(t,m)dif_eq(t, m, theta, u), tspan, m0);
solution = sol(1, :);
end

