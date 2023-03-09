import matplotlib.pyplot as plt
import numpy as np
landmark_0x = []
landmark_0y = []
landmark_1x = []
landmark_1y = []
landmark_2x = []
landmark_2y = []
for i in np.linspace(0, 9450, 50):
    f = open(i + '.txt','r')
    i = 0
    for row in f:
        if (i%6 == 0):
            landmark_0x.append(row)
        if (i%6 == 1):
            landmark_0y.append(row)
        if (i%6 == 2):
            landmark_1x.append(row)
        if (i%6 == 3):
            landmark_1y.append(row)
        if (i%6 == 4):
            landmark_2x.append(row)
        if (i%6 == 5):
            landmark_2y.append(row)
  
plt.plot(np.linspace(0, 9450/50), landmark_0x, label='Landmark 0x')
plt.plot(np.linspace(0, 9450/50), landmark_0y, label='Landmark 0y')
plt.plot(np.linspace(0, 9450/50), landmark_1x, label='Landmark 1x')
plt.plot(np.linspace(0, 9450/50), landmark_1y, label='Landmark 1y')
plt.plot(np.linspace(0, 9450/50), landmark_2x, label='Landmark 2x')
plt.plot(np.linspace(0, 9450/50), landmark_2y, label='Landmark 2y')
plt.xlabel('Time', fontsize = 12)
plt.ylabel('Position', fontsize = 12)
  
plt.title('Landmarks', fontsize = 20)
plt.legend()
plt.show()