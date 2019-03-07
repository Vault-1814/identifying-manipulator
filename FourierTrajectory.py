""" Setup:
    1) spectrum of signal
    2) period of signal
    3) maximum allowed limits
    4)* duration of trajectory

"""
from numpy import *
import matplotlib.pyplot as plt

S0 = [3, 0.5, 3, 0.2, 0.3, 6]
T0 = 5
DURATION = 10


class FourierTrajectory:

    def __init__(self, T, s, allowed_limits=(0.03, 5.79)):
        self.s = s  # spectrum
        self.T = T  # period of signal
        self.w0 = 2 * pi / T
        self.allowed_limits = allowed_limits

        self.a, self.b = [-1], [-1]

        self.q0 = 0
        self.scale = 1

        self.__compute_ab()
        self.__compute_adjust()

    def __del__(self):
        pass

    def get_a(self):
        return self.a

    def get_b(self):
        return self.b

    def __compute_ab(self):
        func = fft.ifft(self.s)
        complex_s = fft.fft(func)
        for s in complex_s:
            psi = arctan2(imag(s), real(s))
            self.a.append(2 * abs(s) * cos(psi))
            self.b.append(-2 * abs(s) * sin(psi))
        print("a_k and b_k are computed!")

    def __compute_adjust(self):
        __, qs, __ = self.get_trajectory(self.T)
        A = amax(qs) - amin(qs)
        A0 = abs(self.allowed_limits[1] - self.allowed_limits[0])
        # TODO: check computation of scale and shift
        self.scale = amin([A, A0]) / amax([A, A0])
        self.q0 = self.allowed_limits[1] - self.scale * amax(qs)

    def get_point(self, t, q0=0, scale=1):
        q, dq = 0, 0
        for k in range(1, len(self.s) + 1):
            q = q0 + q + self.a[k] * cos(k * self.w0 * t) + self.b[k] * sin(k * self.w0 * t)
            dq = dq + self.a[k] * sin(k * self.w0 * t) - self.b[k] * cos(k * self.w0 * t)
        q = scale * q
        dq = scale * dq
        return q, dq

    def get_trajectory(self, duration, dt=0.001, q0=0, scale=1):
        ts, qs, dqs = [], [], []
        for t in arange(0, duration, dt):
            q, dq = self.get_point(t, self.q0, self.scale)
            ts.append(t)
            qs.append(q)
            dqs.append(dq)
        return ts, qs, dqs


if __name__ == '__main__':
    ft = FourierTrajectory(T0, S0)

    ts, qs, dqs = ft.get_trajectory(DURATION)

    print(amax(qs), amin(qs))
    fig1, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
    ax1.bar(arange(0, len(S0)), S0, 0.1)
    ax2.plot(ts, qs)
    plt.show()
