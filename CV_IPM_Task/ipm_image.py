
import numpy as np
import cv2




def do_ipm(image, **kwargs):
    (h, w) = (image.shape[0], image.shape[1])
    ymax = 566
    x1 = w//2 - 386
    x2 = w//2 + 114
    l = 1200
    # Image coordinates after undistortion
    source = np.float32([[24., 539.], [931., 599.], [737., 424.], [361., 426.]])

    # Image coordinates without undistortion
    # source = np.float32([[19., 550.], [1004., 611.], [767., 436.], [374., 431.]])
    destination  = np.float32([[x1, ymax], [x2, ymax], [x2, ymax-l], [x1, ymax-l]])
    M = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(image, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    
    return warped




def undistort( image, camera=1):
    camera_matrix = None
    dist_coeffs = None
    if camera==1: #camera1 parameters - Front Camera
        camera_matrix = np.array([[722.2992, 0,952.2574], [0, 722.1567, 539.8917], [0, 0, 1.000]])
        dist_coeffs = np.array([[-0.2058, 0.0562, 0., 0., 0.]])
    elif camera==0: #camera2 parameters - Left Camera
        camera_matrix = np.array([[841.6960, 0, 617.4311], [0, 840.6221, 388.3993], [0, 0, 1.000]])
        dist_coeffs = np.array([[-0.4017, 0.1730, 0., 0., 0.]])
    elif camera==2: #camera3 parameters - Right Camera
        camera_matrix = np.array([[874.9190, 0, 606.9020], [0, 868.3715, 370.6982], [0, 0, 1.000]])
        dist_coeffs = np.array([[-0.4519, 0.1926, 0., 0., 0.]])
    print(np.shape(image))
    width, height = np.shape(image)[1], np.shape(image)[0] 
    newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width, height), 1, (width, height))
    undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs, None, newcameramatrix)
    return undistorted_image






def straight_lane_detection(image):
    print('Handling Straight Lane')

    # x1, y1, x2, y2, _ = lane_without_stoplines(np.copy(image)) # 2 endpoints of the stopline

    # image[:y1,:] = 0 # region above stopline made empty as it would introduce bias in straight lane prediction
    image=undistort(image)
    image  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert image to grayscale
    image  = cv2.resize(image, (1280, 720), interpolation = cv2.INTER_AREA) # remove the bottom portion of image
    warped_image = do_ipm(image) # get bird's eye view of image
    return warped_image
    # cv2.imwrite(f'/home/ayush/Desktop/ipm_image{i}.png', warped_image)
   # thresholded_image = 255*(((warped_image > thresh1) & (warped_image < thresh2)) | ((warped_image > thresh3) & (warped_image < thresh4)) | ((warped_image > thresh5) & (warped_image < thresh6))).astype('uint8') # binary threshold by 180
   # cv2.imwrite('thresh_img.png', thresholded_image)
   # eroded_image = cv2.erode(thresholded_image, np.ones((5,5), np.uint8), iterations = 2) # erosion done to remove small white noise bubbles
   # cv2.imwrite('eroded_img.png', eroded_image)
   # eroded_image = self.adjust_camera(eroded_image)
   # (h,w) = eroded_image.shape[:2]

    #cv2.imshow("Image_Original", eroded_image)
    #cv2.waitKey(0)

   # reference_x, reference_y = get_reference_lane_straight(thresholded_image)
#  x0, y0, x1, y1, x2, y2, current_lane = generate_all_lanes_straight(h, w, reference_x, reference_y)

  #  plt.plot(x,hist)
        
   # plt.title(f"Image{i} Histogram")
    #cv2.imwrite('left_lane.png',image[:,min(cor_lane[0],cor_lane[1]):max(cor_lane[2],cor_lane[3])])
    #plt.savefig(f"/home/ayush/Desktop/left_lane/left_lane_hist{i}.png")
    #plt.show()


path = '/Users/kshitijvaidya/Desktop/VirtualEnvironment/SeDriCa_Projects/CV_IPM_Task/camera_image1.png'
image= cv2.imread(path)

ipm_image = straight_lane_detection(image)
cv2.imwrite('ipm_test2.png', ipm_image)
