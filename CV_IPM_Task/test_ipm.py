import cv2
import numpy as np 

def do_ipm(image, **kwargs):
    (h, w) = (image.shape[0], image.shape[1])
    ymax = 648
    x1 = w//2 - 364
    x2 = w//2 + 136
    l = 1200
    # Image coordinates after undistortion
    source = np.float32([[8., 594.], [994., 596.], [788., 470.], [380., 474.]])

    # Image coordinates without undistortion
    # source = np.float32([[19., 550.], [1004., 611.], [767., 436.], [374., 431.]])
    destination  = np.float32([[x1, ymax], [x2, ymax], [x2, ymax-l], [x1, ymax-l]])
    M = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(image, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    
    return warped


path = '/Users/kshitijvaidya/Desktop/VirtualEnvironment/SeDriCa_Projects/CV_IPM_Task/camera_image1.png'
image = cv2.imread(path)
warped_image = do_ipm(image)

save_path = 'ipm_test.png'
cv2.imwrite(save_path, warped_image)