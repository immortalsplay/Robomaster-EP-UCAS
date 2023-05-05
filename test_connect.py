from robomaster import robot
from robomaster import config
from robomaster import camera
import cv2
import keyboard
import _thread
import time

if __name__ == '__main__':
    ep_robot = robot.Robot()
    # 指定机器人的 SN 号
    ep_robot.initialize(conn_type="sta", sn="3JKDH3B00118M5")

    ep_version = ep_robot.get_version()
    print("Robot Version: {0}".format(ep_version))


    ep_chassis= ep_robot.chassis
    ep_camera= ep_robot.camera

    ep_camera.start_video_stream(display=False)

    def imshow_thread(a):
        for _ in range(3000):
            img = ep_camera.read_cv2_image()
            cv2.imshow("Robot", img)
            cv2.waitKey(1)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
    
    try:
        _thread.start_new_thread(imshow_thread,(1,))
    except:
        print ("Error: 无法启动线程")
    x_val = 0.5
    y_val = 0.3
    x_val = 0.5
    y_val = 0.3
    z_val = 30

    # 前进 3秒
    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    time.sleep(3)

    # 后退 3秒
    ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=5)
    time.sleep(3)

    # 左移 3秒
    ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5)
    time.sleep(3)

    # 右移 3秒
    ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=5)
    time.sleep(3)

    # 左转 3秒
    ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5)
    time.sleep(3)

    # 右转 3秒
    ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
    time.sleep(3)

    # 停止麦轮运动
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    

    ep_robot.close()