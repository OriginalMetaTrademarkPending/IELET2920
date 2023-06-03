<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function sol = disc_theta_to_ode(phi, N, u, M)
=======
function solution = disc_theta_to_ode(phi, N, u)
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
=======
function solution = disc_theta_to_ode(phi, N, u, M)
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
function sol = disc_theta_to_ode(phi, N, u, M)
>>>>>>> 2393880 (Final updates to the code)
=======
function sol = disc_theta_to_ode(phi, N, u, M)
>>>>>>> master
%The parametric differential equation to be implemented in the objective
%function. Note that only the active muscle mass is measured, the fatigued
%muscle mass is a hidden variable.
solution = NaN(2, N);
solution(:, 1) = zeros(2, 1);
for i = 2:N
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    solution(:, i) = disc_diff_eq(phi, solution(:, i-1), u(i), M);
=======
    solution(:, i) = disc_diff_eq(solution(:, i-1), phi, u(i-1));
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
=======
    solution(:, i) = disc_diff_eq(phi, solution(:, i-1), u(i-1));
>>>>>>> 4f56570 (Changes in the new estimator, will try to finalize tomorrow)
=======
    solution(:, i) = disc_diff_eq(phi, solution(:, i-1), u(i-1), M);
>>>>>>> e10bbb0 (New parameter estimation changes)
=======
    solution(:, i) = disc_diff_eq(phi, solution(:, i-1), u(i-1), M);
>>>>>>> master
end
sol = solution(1, :);
end
