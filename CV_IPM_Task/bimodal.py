import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import rayleigh
import cv2

path = '/Users/kshitijvaidya/Desktop/VirtualEnvironment/CV_IPM_Task/ipm_image/ipm_image0.png'
image = cv2.imread(path)
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
hist = cv2.calcHist([image], [0], None, [255], [1, 256]) / 400000.0
x = np.linspace(0, 60, 255)


# Set random seed for reproducibility
np.random.seed(42)

# Parameters for the first normal distribution
mean1, std_dev1 = 20, 1

# Parameters for the second normal distribution
mean2, std_dev2 = 10, 2

# Generate random samples from the first and second normal distributions
data1 = np.random.rayleigh(mean1, 1000)
data2 = np.random.rayleigh(mean2, 1000) + 22

# Combine the two datasets to create a bimodal distribution
bimodal_data = np.concatenate((data1, data2))

# Plot the histograms of the individual distributions and the combined bimodal distribution
plt.hist(data1, bins=50, density=True, alpha=0.5, label='Distribution 1')
plt.hist(data2, bins=50, density=True, alpha=0.5, label='Distribution 2')
plt.hist(bimodal_data, bins=50, density=True, alpha=0.7, label='Bimodal Distribution')
plt.plot(x, hist)
plt.title('Bimodal Distribution')
plt.xlabel('Value')
plt.ylabel('Density')
plt.legend()
plt.show()
