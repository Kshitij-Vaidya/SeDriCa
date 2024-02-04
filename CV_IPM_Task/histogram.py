import numpy as np
import scipy as sc
import cv2
import matplotlib.pyplot as plt

i = 0
while(i < 99):
    path = f'/Users/kshitijvaidya/Desktop/VirtualEnvironment/CV_IPM_Task/ipm_image/ipm_image{i}.png'
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([image], [0], None, [255], [1, 256])
    plt.plot(hist)
    plt.title(f'Histogram : Image {i}')
    save_path = f'/Users/kshitijvaidya/Desktop/VirtualEnvironment/CV_IPM_Task/Histograms/histogram{i}.png'
    plt.savefig(save_path)
    plt.show()
    i = i + 1