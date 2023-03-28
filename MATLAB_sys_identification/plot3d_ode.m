% This script makes a 3d_plot of the ode for the different guesstimates.
type diff_eq

u = @(t) heaviside(t - 5) - heaviside(t - 15) + heaviside(t - 20) - heaviside(t - 30);
tspan = 0:0.001:30;
theta_real = [2, -1000, 2000, -9];
m0 = [0 0];
soltrue = ode45(@(t, m)diff_eq(t, m, theta_real, u(t)), tspan, m0);
m = deval(soltrue, tspan);
plot(m(1), soltrue.y(1, :))
hold on
plot(m(2), soltrue.y(2, :))
hold off