import cv2
from apriltag_s import Apriltag
#import apriltag as apriltags 
import numpy as np
import tagUtils as tud
import matplotlib.pyplot as plt
import math

tag_point_L = np.array([[(4040-66.5)/1000, (4040-66.5)/1000, 0.0],
                        [(4040+66.5)/1000, (4040-66.5)/1000, 0.0],
                        [(4040+66.5)/1000, (4040+66.5)/1000, 0.0],
                        [(4040-66.5)/1000, (4040+66.5)/1000, 0.0]])
tag_L = [[4040], [3445], [200]]
tag_position = [[tag_L]]


def camdetect(img):
    
    b,g,r = cv2.split(img)
    frame = cv2.merge([r,g,b])
    # low_blue = np.array([70, 65, 57])
    # high_blue = np.array([111, 255, 255])

    # low_blue = np.array([10, 70, 70])
    # high_blue = np.array([75, 255, 255])

    # low_red = np.array([100, 50, 70]) #110
    # high_red = np.array([255, 255, 180])#135

    # low_yellow = np.array([20, 30, 30])
    # high_yellow = np.array([150, 255, 255])

    # low_no_yellow = np.array([0, 60, 50])
    # high_no_yellow = np.array([255, 255, 255])

    low_blue = np.array([10, 70, 70])
    high_blue = np.array([75, 255, 255])

    low_red = np.array([100, 50, 90]) #110
    high_red = np.array([255, 255, 190])#135

    low_yellow = np.array([70, 30, 150])
    high_yellow = np.array([110, 255, 200])

    low_no_yellow = np.array([10, 60, 100])
    high_no_yellow = np.array([150, 210, 195])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_no_yellow = cv2.inRange(hsv, low_no_yellow, high_no_yellow)
    mask_red = cv2.inRange(hsv, low_red, high_red)
    mask_blue = cv2.inRange(hsv, low_blue, high_blue)
    mask_yellow = cv2.inRange(hsv, low_yellow, high_yellow)

    res = cv2.bitwise_and(frame,frame, mask= mask_yellow)
    res = res+255

    plt.imshow(res)
    plt.show()
    detector = Apriltag()
    detector.create_detector(debug=False)
    #test = apriltags.Detector()    
    # # cap = img
    # # cap.set(3,1920)
    # # cap.set(4,1080)
    # # fps = 24
    # # window = 'Camera'
    # # cv2.namedWindow(window)

    # detector = Apriltag()
    # detector.create_detector(debug=False)
    # #test = apriltags.Detector()             
      
    # low_blue = np.array([10, 50, 57])
    # high_blue = np.array([111, 255, 255])

    # # low_blue = np.array([70, 65, 57])
    # # high_blue = np.array([111, 255, 255])

    # low_red = np.array([113, 20, 23]) #110
    # high_red = np.array([255, 255, 230])#135
    
    # low_yellow = np.array([1, 40, 40])
    # high_yellow = np.array([100, 255, 255])
    # aframe = img
    # b,g,r = cv2.split(aframe)
    # frame = cv2.merge([r,g,b]) 
    # plt.imshow(frame)
    # plt.show()
 

    # hist,bins = np.histogram(frame.flatten(),256,[0,256]) 
    # cdf = hist.cumsum() #计算累积直方图
    # cdf_m = np.ma.masked_equal(cdf,0) #除去直方图中的0值
    # cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())#等同于前面介绍的lut[i] = int(255.0 *p[i])公式
    # cdf = np.ma.filled(cdf_m,0).astype('uint8') #将掩模处理掉的元素补为0
    
    # #计算
    # result2 = cdf[frame]
    # # hsv = cdf[frame]
    # # # result = cv2.LUT(frame, cdf)
    # # plt.imshow(result2)
    # # plt.show()

    # blur = cv2.blur(frame,(5,5))
    # blur0=cv2.medianBlur(blur,5)
    # blur1= cv2.GaussianBlur(blur0,(5,5),0)
    # blur2= cv2.bilateralFilter(blur1,9,75,75)
    # hsv = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)
    # # gray = np.array(cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY))
    # # lut = np.zeros(256, dtype = frame.dtype )#创建空的查找表
    # # im1 = np.array(frame)
    # # hist = im1.flatten()
    # plt.imshow(hsv)
    # plt.show()

    # # hsv = img

    # # hsv = np.array(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
    # #f = frame[:,:,1]

    # # detections,contours = detector.detect(frame)
    # # frame = np.copy(frame)
    # # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


    # mask_red = cv2.inRange(hsv, low_red, high_red)
    # mask_blue = cv2.inRange(hsv, low_blue, high_blue)

    # mask_yellow = cv2.inRange(hsv, low_yellow, high_yellow)

    # res = cv2.bitwise_and(frame,frame, mask= mask_blue)
    # res = res+255
    # plt.imshow(res)
    # plt.show()
    # r,g,b = cv2.split(frame)
    # frame = cv2.merge([b,g,r]) 
    detections,contours = detector.detect(frame)
    print('detections',detections)
    cv2.drawContours(res,contours,-1,(0,0,255),3)
    # cv2.circle(res, tuple([0].astype(int)), 4,(255,0,255), 2)
    # cv2.line(res,tuple(contours[0]),tuple(contours[1]),(255,0,255),5)                   
    #tests = test.detect(gray)
    print('contours',contours)
    # cv2.imwrite(r"E:\1lab materials\robomaster\RMYC\testcam2.jpg", res)
    # cv2.imwrite(r"E:\1lab materials\robomaster\RMYC\testcam3.jpg", frame)
    #testpoint = tuple(tests.corners[0])
    show = None
    if (len(detections) == 0):
        show = frame
        print('nope')
        cv2.imwrite(r"E:\1lab materials\robomaster\RMYC\testcam.jpg", res)
        cv2.imwrite(r"E:\1lab materials\robomaster\RMYC\testcam1.jpg", frame)
    else:
        show = frame
        edges = np.array([[0, 1],
                            [1, 2],
                            [2, 3],
                            [3, 0]])
        

        for detection in detections:
            point,rvec_cam,tvec_cam = tud.get_pose_point(detection.homography,tag_point_L)
            # point,rvec_cam,tvec_cam = tud.get_pose_point(contours,tag_point_L)
            dis = tud.get_distance(contours,tag_point_L,122274)
            for j in range(4):
                cv2.line(res,tuple(point[edges[j,0]]),tuple(point[edges[j,1]]),(255,0,255),10)

        cv2.imwrite(r"E:\1lab materials\robomaster\RMYC\testcam.jpg", res)
        cv2.imwrite(r"E:\1lab materials\robomaster\RMYC\testcam1.jpg", frame)
        print ('detection.id:' , detection.id)

        #for tag in tests:
        #        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4,(0,0,255), 2) # right-bottom
            #       cv2.circle(frame,tuple(tag.corners[1].astype(int)), 4,(0,0,255), 2) # left-top
            #       cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4,(0,0,255), 2) # right-top
            #      cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4,(0,0,255), 2) # left-bottom
            #      cv2.line(frame,tuple(tag.corners[0].astype(int)),tuple(tag.corners[1].astype(int)),(0,255,0),2)
            #      cv2.line(frame,tuple(tag.corners[1].astype(int)),tuple(tag.corners[2].astype(int)),(0,255,0),2)
            #       cv2.line(frame,tuple(tag.corners[2].astype(int)),tuple(tag.corners[3].astype(int)),(0,255,0),2)
            #       cv2.line(frame,tuple(tag.corners[3].astype(int)),tuple(tag.corners[0].astype(int)),(0,255,0),2)
        # print ('point:' , point)
        #print ('testpoint:' , testpoint)
    ########################
    
    num_detections = len(detections)
    if num_detections==0:
        print("no")
        return 
    # cv2.imshow(res)
    # plt.show()
    # k = cv2.waitKey(1000//int(fps))

    # if k == 27:
    #     break
# cap.release()
    # print('dis',dis)


    print('detection.id',detection.id)
    print('num_detections',num_detections)
    print('rvec_cam',rvec_cam)
    print('tvec_cam',tvec_cam)
    return
    # return detection.id,num_detections,rvec_cam,tvec_cam

filename = (r"E:\1lab materials\robomaster\RMYC\123.jpg")
frame = cv2.imread(filename)
if __name__ == '__main__':
    # camdetect(frame)
    # id,num_detections,rvec_cam,tvec_cam = camdetect(frame)
    # nav_planner.cam_callback(id,num_detections,rvec_cam,tvec_cam)

    x = [ 20, 70, 110, 160, 169, 175, 500, 1000, 2000, 2500, 5000, 12000, 16000, 16900, 18000, 20000, 30000, 40000]
    y = [2.48,0.884,0.569,0.394,0.374,0.36,0.126,0.0633,0.0316,0.0254,0.0127,0.00527,0.00394,0.00377,0.00352,0.00317,0.00211,0.00158]
    y=[math.log(2.48,10),math.log(0.884,10),math.log(0.569,10),math.log(0.394,10),math.log(0.374,10),math.log(0.36,10),math.log(0.126,10),math.log(0.0633,10),math.log(0.0316,10),math.log(0.0254,10),math.log(0.0127,10),math.log(0.00527,10),math.log(0.00396,10),math.log(0.00377,10),math.log(0.00352,10),math.log(0.00317,10),math.log(0.00211,10),math.log(0.00158,10)]
    plt.plot(x,y)
    plt.show()