% We have 5 parameters, defining them as symbols
syms p_af p_ar p_ra p_fa M

% Defining the matrices for u = 0 and u = 1
A_0 = [p_af - p_ar, 1 - p_fa;
    1 - p_af, p_fa];

A_1 = [p_af - p_ra, 1 - p_fa - p_ra;
    1 - p_af, p_fa];

%Eigenvalues for both cases
eig_0 = eig(A_0);
eig_1 = eig(A_1);

eig_0 = simplify(eig_0);

disp(eig_0)
disp(eig_1)

sym_vec = [p_af; p_ar; p_ra; p_fa];

solve(eig_1(1) <= 1, p_ra)
solve(eig_1(2) <= 1, p_af)
solve(eig_0(1) <= 1, p_ar)
solve(eig_1(2) <= 1, p_fa)