import matplotlib.pyplot as plt
from numpy import arange, zeros, sin, pi, cos

N = 5

joints_min = [0.025, 0.025, -5.0, 0.025, 0.12]
joints_max = [5.80, 2.6, -0.025, 3.4, 5.64]

jointsA = [abs(joints_max[i] - joints_min[i]) for i in range(N)]
jointsAd2 = [jointsA[i] / 2 for i in range(N)]


nf = [8, 8, 8, 8, 8]
T = [9, 6, 7, 4.5, 6]

a = [  # 1    2      6    4     5      6     7     8
    [-1, -3.00, 0.50, 0.10, 1.00, 0.10, 0.50, 0.10, 3.00],
    [-1, 2.00, 0.10, 1.00, 0.10, 0.00, 0.00, 0.00, 1.50],
    [-1, -3.00, 1.00, 0.50, 1.00, 0.50, 1.00, 1.00, 6.00],
    [-1, -2.00, 0.10, 0.10, 1.00, 0.10, 0.02, 0.10, 1.00],
    [-1, 3.00, 0.10, 1.00, 0.02, 0.10, 0.02, 0.10, 2.00]
]


scale = [0.39, 0.29, 0.2, 0.42, 0.45]
q0 = [jointsAd2[0] - 0.4, jointsAd2[1] - 0.13, -jointsAd2[2] - 0.3, jointsAd2[3] - 0.3, jointsAd2[4] - 0.3]

f = open('trajectories.txt', 'w')


L = 0
ts, qs, dqs = [], [], []
t = 0
while t < 10:
    q, dq = zeros(N), zeros(N)
    for i in range(N):
        w0 = 2 * pi / T[i]
        for k in range(1, nf[i] + 1):
            q[i] = q[i] + a[i][k] * cos(k * w0 * t)
            dq[i] = dq[i] - a[i][k] * sin(k * w0 * t)
        q[i] = q0[i] + scale[i] * q[i]
        dq[i] = scale[i] * dq[i]

    t = t + 0.01

    f.write("{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(t, q[0],q[1],q[2],q[3],q[4], dq[0],dq[1],dq[2],dq[3],dq[4],
                      joints_max[0],joints_max[1],joints_max[2],joints_max[3],joints_max[4],
                      joints_min[0],joints_min[1],joints_min[2],joints_min[3],joints_min[4]))

    ts.append(t)
    qs.append(q[L])
    dqs.append(dq[L])

f.close()

plt.plot(ts, qs)
plt.plot(ts, dqs)
plt.plot(ts, [joints_max[L] for j in range(len(ts))], 'r--')
plt.plot(ts, [joints_min[L] for j in range(len(ts))], 'r--')
plt.show()


# q = 0
# ts = []
# qs = []
# dqs = []
# a = [0, 1.4, 0.1, 0.1, 0.02, 0.1, 0.02, 0.1, 0.2]
# w0 = 2 * pi / 4
# nf = 9
# q0 = 2.5
#
#
# for t in arange(0, 10, 0.01):
#     q = 0
#     dq = 0
#     for k in range(nf):
#         q = q + a[k] * cos(k*w0*t)
#         dq = dq - a[k] * sin(k*w0*t)
#     q = q + q0
#     ts.append(t)
#     qs.append(q)
#     dqs.append(dq)

#
# plt.plot(ts, qs)
# plt.plot(ts, dqs)
# plt.plot(ts, [0 for i in range(len(ts))], 'r--')
# plt.show()
