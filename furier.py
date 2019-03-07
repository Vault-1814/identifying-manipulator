from numpy import *

f = open('furier.txt', 'w')

n = 5

nf = 5

T = 4
w0 = 2 * pi / T

lower = [0, 0, -5.18, 0, 0]
upper = [5.9, 2.71, 0, 3.58, 5.85]

qs = [0, 0, 0, 0, 0]

for i in range(n):
    qs[i] = upper[i] - lower[i]



a = [2.5, 1.25, 0.7, 0.35, 0.15]
b = [0, 0.1, 0.2, 0.3, 0.4]

for k in range(nf):
    q = q + a[k] * sin(k * w0 * t) + b[k] * cos(k * w0 * t)



f.close()
