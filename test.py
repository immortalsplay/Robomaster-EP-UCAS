import cv2
from apriltag_s import Apriltag
import matplotlib.pyplot as plt
import numpy as np
import time
import tagUtils as tud

tag_L = np.array([[4040-66.5, 4040-66.5, 0.0],
                        [4040+66.5, 4040-66.5, 0.0],
                        [4040+66.5, 4040+66.5, 0.0],
                        [4040-66.5, 4040+66.5, 0.0]])


# time_start = time.time()
ap = Apriltag()
ap.create_detector(debug=True)
filename = (r"E:\1lab materials\img\1\001.jpg")
frame = cv2.imread(filename)
# b,g,r = cv2.split(frame)
# frame = cv2.merge([r,g,b])

low_blue = np.array([70, 65, 57])
high_blue = np.array([111, 255, 255])

low_red = np.array([70, 20, 110]) #110C
high_red = np.array([255, 255, 135])#135

low_yellow = np.array([20, 30, 30])
high_yellow = np.array([150, 255, 255])

hist,bins = np.histogram(frame.flatten(),256,[0,256]) 
cdf = hist.cumsum() #计算累积直方图
cdf_m = np.ma.masked_equal(cdf,0) #除去直方图中的0值
cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())#等同于前面介绍的lut[i] = int(255.0 *p[i])公式
cdf = np.ma.filled(cdf_m,0).astype('uint8') #将掩模处理掉的元素补为0

#计算
result2 = cdf[frame]
# result = cv2.LUT(frame, cdf)
frame = result2
blur = cv2.blur(frame,(5,5))
blur0=cv2.medianBlur(blur,5)
blur1= cv2.GaussianBlur(blur0,(5,5),0)
blur2= cv2.bilateralFilter(blur1,9,75,75)
hsv = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)

mask_red = cv2.inRange(hsv, low_red, high_red)
mask_blue = cv2.inRange(hsv, low_blue, high_blue)
mask_yellow = cv2.inRange(hsv, low_blue, high_blue)

res = cv2.bitwise_and(frame,frame, mask= mask_yellow)
res = res+255

detections,contours = ap.detect(res)
print('contours',contours)
for detection in detections:
    point,rvec_cam,tvec_cam = tud.get_pose_point(detection.homography,tag_L)
    dis = tud.get_distance(detection.homography,tag_L,122274)

    # dis = tud.get_distance(detection.homography,30501)
print('point',point)
print('rvec_cam',rvec_cam)
print('tvec_cam',tvec_cam)
time_end = time.time()
if len(detections) > 0:
    print('识别成功')
else:
    print('识别失败') 
# print('contours cost', time_end - time_start)