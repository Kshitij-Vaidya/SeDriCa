import cv2
import numpy as np

def apply_sobel(image):
   
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
   
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    
    
    sobel_combined = np.sqrt(sobelx**2 + sobely**2)
    
    # Normalize the result to 0-255
    sobel_normalized = cv2.normalize(sobel_combined, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    
    return sobel_normalized



# Change range as per dataset -- works for all IPM images
for i in range(95, 106):
    path = f'/Users/kshitijvaidya/Downloads/final_dataset/saved_ipm/ipm_image{i}.png'
    img = cv2.imread(path)
    sobel_result = apply_sobel(img)

    j = 0
    for j in range(780):
        k = 0
        while sobel_result[j,k] == 0:
            k+=1

        for l in range(10):
            sobel_result[j,k+l] = 0

    for j in range(780):
        k = 1279
        while sobel_result[j,k] == 0:
            k-=1
        for l in range(10):
            sobel_result[j,k-l] = 0

    out_path = f'sobel_img{i}.png'
    cv2.imwrite(out_path, sobel_result)