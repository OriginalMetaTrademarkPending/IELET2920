import numpy as np
import matplotlib.pyplot as plt

x = np.array([1, 2, 4])
y = np.array([2, 3, 7])

X = np.array([np.ones(x.size), x, x*x])
Y = y

theta = np.dot(Y, np.linalg.pinv(X))

f = lambda x : theta[0] + theta[1]*x + theta[2]*x*x

xx = np.linspace(0, 5, 100)
yy = f(xx)

plt.plot(x, y, 'ro', label = 'DataPoints')
plt.plot(xx, yy, 'b', label = 'Fitted Equation')
plt.legend()
plt.show()