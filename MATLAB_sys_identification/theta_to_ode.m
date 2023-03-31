function solution = theta_to_ode(theta, tspan, m0, u)
%The parametric differential equation to be implemented in the objective
%function. Note that only the active muscle mass is measured, the fatigued
%muscle mass is a hidden variable.
sol = ode45(@(t,m)diff_eq(t, m, theta, u(t)), tspan, m0);
solpts = deval(sol, tspan);
solution = solpts(1, :)';
end

