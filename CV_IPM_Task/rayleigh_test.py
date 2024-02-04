import numpy as np
from scipy.stats import rayleigh
import cv2
import matplotlib.pyplot as plt
import math

path = '/Users/kshitijvaidya/Desktop/VirtualEnvironment/CV_IPM_Task/ipm_image/ipm_image0.png'
image = cv2.imread(path)
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
hist = cv2.calcHist([image], [0], None, [207], [50, 256]) 
hist_normal = (hist - np.min(hist)) / (np.max(hist) - np.min(hist)) 
left_hist = hist_normal[0:97]
right_hist = hist_normal[97:]

x = np.linspace(0, 1, 207)

mean1 = np.mean(left_hist)
mean2 = np.mean(right_hist)

param1 = mean1 * math.sqrt(2 / math.pi)
param2 = mean2 * math.sqrt(2 / math.pi)

data1 = np.random.rayleigh(param1, 5000) 
data2 = np.random.rayleigh(param2, 2000) + 0.5

concat_data = np.concatenate((data1, data2))

print(param1, param2)


plt.hist(data1, bins = 40, label="Data1", density = True, alpha=0.2, color='red')
plt.hist(data2, bins = 40, label="Data2", density = True, alpha=0.2, color='blue')
plt.hist(concat_data, bins=40, label="Concat", density = True, alpha = 0.8, color='green')

hist_normal = (hist - np.min(hist)) / (np.max(hist) - np.min(hist)) * 0.6
plt.plot(x, (hist_normal * 3), color='black')
plt.xlabel('X')
plt.ylabel('Probability Density')
plt.title('Rayleigh Distribution PDF')
plt.legend()

# plt.plot(concat_data)
plt.show()

