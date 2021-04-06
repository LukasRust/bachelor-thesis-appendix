import numpy as np
import matplotlib.pyplot as plt
import csv
import math

t_g = 10.
k = 0.5
t = 0.
ts = []
ws = []
w_dots = []
ts.append(t)
w_dot = 0.
w = 0.
ws.append(w)
w_dots.append(w_dot)


def e(time, k, t_g):
    if time < t_g/2. :
        return 1. - math.exp(-k * time)
    else:
        return max(0.01,1. - math.exp(-k * (t_g - w)))

while w < t_g:
    print(w_dot)
    t = t + 0.2
    ts.append(t)
    w_dot = e(t,k,t_g)
    w = w + w_dot * 0.2
    w_dots.append(w_dot)
    ws.append(w)

#plt.axis('equal')
plt.plot(ts, ws, 'b-')
plt.xlabel("t")
plt.ylabel("w")
plt.show()

plt.plot(ts, w_dots, 'b-')
plt.xlabel("t")
plt.ylabel("w_dot")
plt.show()
