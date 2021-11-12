import numpy as np
import matplotlib.pyplot as plt
Xh = []
theta = 0.1
L = 2.0
x0 = L*np.sin(theta)
y0 = L*np.cos(theta)
dtdt = 0.1
vx0 = L*np.cos(theta)*dtdt
vy0 = -L*np.sin(theta)*dtdt

X = np.array([x0, -y0])
l0 = np.linalg.norm(X)
Ks = 5
Kd = 0.001*0
m = 0.2038735983690112
V = np.array([vx0, vy0])
print(X, V)
Xh += [X.tolist()]
n = 50000
dt = 0.002
for i in range(n):

    L = np.linalg.norm(X)
    s = L-l0
    r = X/L

    V += (-r*(s*Ks+Kd*np.dot(V, r))+np.array([0, -9.81*m]))*dt/m
    X += V*dt
    Xh += [X.tolist()]

Xh = np.array(Xh)
plt.plot(*Xh.T)
plt.grid()
plt.show()
