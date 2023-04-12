function sol = disc_theta_to_ode(phi, N, u)
%The parametric differential equation to be implemented in the objective
%function. Note that only the active muscle mass is measured, the fatigued
%muscle mass is a hidden variable.
solution = NaN(2, N);
solution(:, 1) = zeros(2, 1);
for i = 2:N
    solution(:, i) = disc_diff_eq(solution(:, i-1), phi, u(i));
end
sol = solution(1, :);
end
