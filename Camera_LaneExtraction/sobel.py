import cv2
import numpy as np
import matplotlib as plt

def apply_sobel(image):
   
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
   
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    
    
    sobel_combined = np.sqrt(sobelx**2 + sobely**2)
    
    # Normalize the result to 0-255
    sobel_normalized = cv2.normalize(sobel_combined, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    
    return sobel_normalized


'''''
def main():
    i=1
    while(i<1861):

        image_path = f'/home/ayush/Desktop/final_dataset/saved_ipm/ipm_image{i}.png'
        original_image = cv2.imread(image_path)

        if original_image is None:
            print("Error: Image not found.")
            return

        # Apply Sobel filter
        sobel_result = apply_sobel(original_image)
  
        output_path = f'/home/ayush/Desktop/final_dataset/sobel_ipm/sobel_ipm{i}.png'  
        cv2.imwrite(output_path, sobel_result)
        i+=1

if __name__ == "__main__":
    main()
'''

path = '/Users/kshitijvaidya/Downloads/final_dataset/saved_ipm/ipm_image162.png'
orig_image = cv2.imread(path)
sobel_result = apply_sobel(orig_image)

out_path = 'sobel_img.py'
cv2.imwrite(out_path, sobel_result)
