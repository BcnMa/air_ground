#/usr/bin/env python
import matplotlib.pyplot as plt


class HighPassFilter:
    def __init__(self, alpha):
        self.last_input = 0.0
        self.last_output = 0.0
        self.alpha = alpha

    def filter(self, _input):
        output = self.alpha * self.last_output + self.alpha * (_input - self.last_input)
        self.last_output = output
        self.last_input = _input
        return output


class LowPassFilter:
    def __init__(self, alpha):
        self.last_output = 0.0
        self.alpha = alpha
    
    def filter(self, _input):
        output = self.alpha * _input + (1-self.alpha) * self.last_output
        self.last_output = output
        return output


lf = LowPassFilter(0.1)

# fig1 = plt.figure()
# data = [lf.filter(10) for i in range(50)]
# plt.plot(range(len(data)), data)
# plt.show()

hf = HighPassFilter(0.9)
fig2 = plt.figure()
data = [hf.filter(10) for i in range(50)]
plt.plot(range(len(data)), data)
plt.show()
