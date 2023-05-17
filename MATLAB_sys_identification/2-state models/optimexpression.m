function [xkp1] = optimexpression(optimvariable, xk, u)
% Optimization expression to be given to fcn2optimexpr
xkp1 = disc_diff_eq(optimvariable, xk, u);
end

