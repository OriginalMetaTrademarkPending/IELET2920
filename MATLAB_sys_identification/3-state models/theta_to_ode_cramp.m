function solution = theta_to_ode_cramp(theta, tspan, m0, u, N, b)
%The parametric differential equation to be implemented in the objective
%function. Note that only the active muscle mass is measured, the fatigued
%muscle mass is a hidden variable.
% sol = ode45(@(t,m)diff_eq(t, m, theta, u(t)), tspan, m0);
% solpts = deval(sol, tspan);
% solution = solpts(1, :)';
sol_timeframe = NaN(3, N);
sol_timeframe(:, 1) = m0';
for i = 2:N
    part_sol = ode45(@(t,m)diff_eq_cramp(t, m, theta, u(i)), [tspan(i-1), tspan(i)], sol_timeframe(:, i-1));
    sol_timeframe(:, i) = part_sol.y(:, end);
end
solution = (sol_timeframe(1, :) + (b*sol_timeframe(3, :)))';
end

