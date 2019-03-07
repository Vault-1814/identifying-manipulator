from numpy import *
import matplotlib.pyplot as plt


class FourierTrajectory:

    def __init__(self, T, s):
        self.s = s  # spectrum
        self.T = T  # period of signal
        self.w0 = 2 * pi / T

        self.a, self.b = [-1], [-1]

        self.__computeAB()

    def __del__(self):
        pass

    def getA(self):
        return self.a

    def getB(self):
        return self.b

    def __computeAB(self):
        func = fft.ifft(self.s)
        complex_s = fft.fft(func)
        for s in complex_s:
            psi = arctan2(imag(s), real(s))
            self.a.append(2 * abs(s) * cos(psi))
            self.b.append(-2 * abs(s) * sin(psi))
        print("a_k and b_k are computed!")


    def getPoint(self, t, q0=0, scale=1):
        q, dq = 0, 0
        for k in range(1, len(self.s)+1):
            q = q + self.a[k] * cos(k * self.w0 * t) + self.b[k] * sin(k * self.w0 * t)
            dq = dq + self.a[k] * sin(k * self.w0 * t) - self.b[k] * cos(k * self.w0 * t)
        return q, dq

    def getTrajectory(self, duration, dt=0.001):
        ts, qs, dqs = [], [], []
        for t in arange(0, duration, dt):
            q, dq = ft.getPoint(t)
            ts.append(t)
            qs.append(q)
            dqs.append(dq)
        return ts, qs, dqs

if __name__ == '__main__':
    spectrum = [3, 0.5, 3, 0.2, 0.3, 6]
    T = 5
    ft = FourierTrajectory(T, spectrum)

    ts,qs,dqs = ft.getTrajectory(10)

    fig1, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
    ax1.bar(arange(0, len(spectrum)), spectrum, 0.1)
    ax2.plot(ts, qs)
    plt.show()
