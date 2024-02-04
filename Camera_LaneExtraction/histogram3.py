import numpy as np
import cv2
import matplotlib.pyplot as plt
import math

cor_lane=[]
def click_event(event, x, y, flags, params):
   if event == cv2.EVENT_LBUTTONDOWN:
      print(f'({x},{y})')
      cor_lane.append(x)
      print(cor_lane)
      cv2.putText(image, f'({x},{y})',(x,y),
      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
      
      
      cv2.circle(image, (x,y), 3, (0,255,255), -1)




x = np.linspace(200, 225, 26)

mean_list = []
std_dev_list = []

i=0
while(i<103):
    image= cv2.imread(f'/Users/kshitijvaidya/Desktop/VirtualEnvironment/Camera_LaneExtraction/IPM_Images/IPM_Image{i}.png')
    
    cv2.namedWindow('Point Coordinates')        
    cv2.setMouseCallback('Point Coordinates', click_event)

    # display the image
    while True:
        cv2.imshow('Point Coordinates',image)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()
    image= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([image[:,min(cor_lane[0],cor_lane[1]):max(cor_lane[2],cor_lane[3])]],[0],None,[26],[200,226])

    hist_arr = hist.reshape(1, -1)
    x_new = x.reshape(1, -1)

    mean = np.sum((hist_arr * x_new)) / (np.sum(hist_arr))
    std_dev = math.sqrt(np.sum((x_new - mean) ** 2) / x_new.shape[1])
    i = i + 1

    mean_list.append(mean)
    std_dev_list.append(std_dev)

    print(hist_arr)
    print(x_new)
    print(hist_arr*x_new)
    print(np.sum(hist_arr))
    print(mean)
    print(std_dev)

    f = open("ValsMeanStdDevLeft.txt", 'a')
    f.write(str(mean) + " " + str(std_dev)+"\n")
    f.close()

    plt.plot(x, hist)
    plt.title(f"Image{i} Histogram")
    cv2.imwrite('right_lane.png',image[:,min(cor_lane[0],cor_lane[1]):max(cor_lane[2],cor_lane[3])])
    plt.savefig(f"/Users/kshitijvaidya/Desktop/VirtualEnvironment/Camera_LaneExtraction/RightLane/right_image{i}")
    plt.show()



print(mean_list)
print(std_dev_list)
    





