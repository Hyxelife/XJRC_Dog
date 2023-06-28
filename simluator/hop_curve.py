# 把这段代码转成python
T = 0.2
t = [i/1000 for i in range(int(T*1000)+1)]
x0 = -10
z0 = -31
x1 = -25
z1 = -27
x2 = -30
z2 = 1
x3 = 5
z3 = -20
print("\n\n\n\n\n")

X = [x0*(1-i/T)**3 + 3*x1*i/T*(1-i/T)**2 + 3*x2*(i/T)**2*(1-i/T) + x3*(i/T)**3 for i in t]
Z = [z0*(1-i/T)**3 + 3*z1*i/T*(1-i/T)**2 + 3*z2*(i/T)**2*(1-i/T) + z3*(i/T)**3 for i in t]
for i in range(len(X)):
    if (X[i]**2 + Z[i]**2)**0.5 > 0.38:
        print("so far")
        print(f"x:{X[i]},z:{Z[i]}")
    if (X[i]**2 + Z[i]**2)**0.5 < 0.15:
        print("so close")
        print(f"x:{X[i]},z:{Z[i]}")

import matplotlib.pyplot as plt
plt.plot(X,Z)
plt.show()