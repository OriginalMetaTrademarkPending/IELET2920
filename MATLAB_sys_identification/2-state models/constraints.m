% This script tries to compute the constraints for the muscle mass system 
% described in the bachelor thesis.

% First, define the state-space in symbolic form
syms phi_af phi_ar phi_ra phi_fa M
% Define the Laplace operator
syms s
F_0 = [phi_af-phi_ar, 1-phi_fa;
    1 - phi_af, -phi_fa];       % State transition function for u = 0
F_1 = [phi_af-phi_ra, 1-phi_fa-phi_ra;
    1 - phi_af, -phi_fa];       % State transition function for u = 1
H = [1 0];  % Measurement function
B = [phi_ra*M; 0];  % Input function

G_0 = H*inv(s*eye(2) - F_0)*B;
G_1 = H*inv(s*eye(2) - F_1)*B;

H_0 = simplify(G_0/(1+G_0));
H_1 = simplify(G_1/(1+G_1));

disp(H_0)
disp(H_1)