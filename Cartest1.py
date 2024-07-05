import time

import numpy as np
from rknn1 import *
import cv2

# rknn_wk = rknn.rknn(r"/home/radxa/Desktop/demo_python/wk.rknn", 0.5, 0.5)
# rknn_sz = rknn.rknn(r"/home/radxa/Desktop/demo_python/sz.rknn", 0.5, 0.5)


def qr_scan(frame):
    """

    :param frame: 传入图像
    :return: 二维码信息
    """
    qrcode = cv2.QRCodeDetector()  # 载入文件库
    QR, points, code = qrcode.detectAndDecode(frame)  # 对二维码进行解码，返回二维码的信息
    if code is None:  # 如果code返回值是none说明没有识别到二维码
        return None, None
    if code is not None:  # 如果code有返回值说明识别到二维码
        return [int(QR[0]), int(QR[1]), int(QR[2])], [int(QR[4]), int(QR[5]), int(QR[6])]


def target_scan_by_color(self, target_color, mod):
    """
    查找目标颜色的坐标偏移值
    :param target_color: 目标颜色 同信息码
    :param mod: “wk” 扫描物块 “sz”扫描十字
    :return: 目标颜色的坐标偏移值
    """
    center = self.center_wk if mod == "wk" else self.center_sz
    while True:
        boxes, classes = self.rknn_wk.rknn_detect(frame=self.frame) if mod == "wk" else self.rknn_sz.rknn_detect(
            frame=self.frame)
        pos = find_target_color(boxes, classes, center, target_color)
        if pos is not None:
            print(f"target_color_scan{pos}")
            # return angle, dis[1]
            return pos


def find_target_color(boxes, classes, center, target_color):
    """

    :param boxes: rknn 返回值 目标位置 左上X,Y右下X,Y 
    :param center: 目标中心点
    :param classes: rknn返回值 目标序列号
    :param target_color:  寻找的目标颜色
    :return:  type: list 与center的偏差值X,Y
    """""

    if boxes is None:
        return None

    if np.sum(np.isin(classes, target_color - 1)) != 1:
        return None

    color_index = np.where(classes == target_color - 1)[0][0]
    x1, y1, x2, y2 = boxes[color_index]
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    centers = [center_x, center_y]
    dx = int(center[0] - centers[0])
    dy = int(center[1] - centers[1])

    return [dx, dy]



def target_scan_by_color(target_color, mod,frame):
    """
    查找目标颜色的坐标偏移值
    :param target_color: 目标颜色 同信息码
    :param mod: “wk” 扫描物块 “sz”扫描十字
    :return: 目标颜色的坐标偏移值
    """
    while True:
        boxes, classes = rknn_wk.rknn_detect(frame=frame) if mod == "wk" else rknn_sz.rknn_detect(
            frame=frame)
        pos = find_target_color(boxes, classes, (320,240), target_color)
        if pos is not None:
            print(f"target_color_scan{pos}")
            # return angle, dis[1]
            return pos




if __name__ == "__main__":

    flagcam1=False
    flagcam2=False
    rknn1 = rknn("wk.rknn", 0.6, 0.6)
    rknn2 = rknn("rknn_sz1.rknn", 0.6, 0.6)

    uart=Uart()
    width,height=640,480
    cam1=cv2.VideoCapture(2)

    # rknn1 = rknn("wk.rknn", 0.6, 0.6)
    if cam1.isOpened():
        flagcam1=True
        print("cam1 ready")
    cam2=cv2.VideoCapture(0)
    cam2.set(3,width)
    cam2.set(4,height)
    if cam2.isOpened():
        flagcam2=True
        print("cam2 ready")
    if flagcam1:
        while True:
#            break
            _, img = cam1.read()
            image = cv2.resize(img, (width, height), interpolation=cv2.INTER_LINEAR)
            order1, order2 = qr_scan(image)

            if order1 is not None:
                print(order1)
                print(order2)
                uart.uart_send_order(order1[0], order1[1], order1[2], order2[0], order2[1], order2[2])
                break

            cv2.imshow("img", img)
            cv2.waitKey(1)
    if flagcam2:
        while True:
            task,color=uart.wait_for_data_packet()
            #task,color=2,1
            print(task)
            print("++")
            print(color)
            if task==1:
                preX,preY=0,0
                cam2.release()
                cam2=cv2.VideoCapture(0)
                cam2.set(3,width)
                cam2.set(4,height)
                if cam2.isOpened():
                    flagcam2=True
                    print("cam2 ready2")
                
                for i in range(20):
                        _,img=cam2.read()
                while True:
                    _,img=cam2.read()
                    for i in range(5):
                        _,img=cam2.read()
                    boxs, scores, classes = rknn1.rknn_detect(img)

                    if scores is not None:

                        if scores>0.78and classes==color:
                            top, left, right, bottom = boxs
                            top = int(top)
                            left = int(left)
                            right = int(right)
                            bottom = int(bottom)
                            centerx=int((left+right)/2)
                            centery=int((top+bottom)/2)
                            print("last")
                            print(f"pre+ X:{preX},Y:{preY}")
                            print(f"now+ X:{centerx},Y:{centery}")
                            preX=centerx
                            preY=centery
                            

                            if (abs(centerx - preX)+abs(centery - preY))<30:
                                print("yes")
                                uart.uart_send_yes()
                                break
                            time.sleep(0.7)


            if task==2:
                count=0
                if color == 1:
                    color = 2
                elif color == 2:
                    color = 1
                cam2.release()
                cam2=cv2.VideoCapture(0)
                cam2.set(3,width)
                cam2.set(4,height)
                if cam2.isOpened():
                    flagcam2=True
                    print("cam2 ready2")
                #time.sleep(0.5)

                # preX,preY=0,0
                #_,img=cam2.read()
                #time.sleep(0.1)
                #_,img=cam2.read()
                for i in range(10):
                    _,img=cam2.read()
                while True:
                    _,img=cam2.read()
                    for i in range(3):
                        _,img=cam2.read()
                    boxs, scores, classes = rknn2.rknn_detect(img,color)
                    if scores is not None:
                        # print("yes")
                        # uart.uart_send_yes()
                        if scores>0.75and classes==color:
                            left,top, right, bottom = boxs
                            top = int(top)
                            left = int(left)
                            right = int(right)
                            bottom = int(bottom)
                            centerx=(float)(left+right)/2
                            centery=(float)(top+bottom)/2
                            # preX=centerx
                            # preY=centery
                            # if abs(centerx - 320)+abs(centery - 240)<50:
                            # if abs(centerx - preX)+abs(centery - preY)<50:
                            #     print("yes task2")
                            dx=centerx - 320
                            dy=centery - 320
                            print(f"center{centerx}++{centery}")
                            print(f"top{top} left{left} right{right} bottem{bottom}")

                            print(f"{dx}++{dy}")
                            flagxy=uart.car_move_xy_cm(dx,dy)
                            #flagxy=0
                            flagcount=0
                            if flagxy:
                                while True:
                                    _1,_2=uart.wait_for_data_packet(1)
                                    if _1:
                                        break
                                    else:
                                        flagxy=uart.car_move_xy_cm(dx,dy)
                                        flagcount+=1
                                        if flagcount==6:
                                            break
                                break
                                #uart.uart_send_yes()
                                
                            
                                # print(f"classNUM:{classes}")
                                # print(f"dX:{dx}")
                                # print(f"dY:{dy}")

                            
                            #cv2.waitkey(300)
                            time.sleep(0.5)
            #break
                           # preX, preY = 0, 0
                            # break

                    # pos=target_scan_by_color(color,"sz",image)
                    # if pos[0] is not None:
                    #     # if abs(pos[0] - 320)+abs(pos[1] - 240)<50:
                    #     #     uart.car_move_xy_cm(1,1)
                    #     break

            if task==3:
                # preX,preY=0,0
                cam2.release()
                cam2=cv2.VideoCapture(0)
                cam2.set(3,width)
                cam2.set(4,height)
                if cam2.isOpened():
                    flagcam2=True
                    print("cam2 ready3")
                time.sleep(0.5)
                for i in range(10):
                    _,img=cam2.read()
                while True:
                    _,img=cam2.read()
                    for i in range(3):
                        _,img=cam2.read()

                    boxs, scores, classes = rknn1.rknn_detect(img,color)
                    if scores is not None:
                        # print("yes")
                        # uart.uart_send_yes()
                        if scores>0.92 and classes==color:
                            left,top, right, bottom = boxs
                            top = int(top)
                            left = int(left)
                            right = int(right)
                            bottom = int(bottom)
                            centerx=(float)(left+right)/2
                            centery=(float)(top+bottom)/2
                            # preX=centerx
                            # preY=centery
                            # if abs(centerx - 320)+abs(centery - 240)<50:
                            # if abs(centerx - preX)+abs(centery - preY)<50:
                            #     print("yes task2")
                            dx=centerx - 320
                            dy=centery - 320
                            print(f"center{centerx}++{centery}")
                            print(f"top{top} left{left} right{right} bottem{bottom}")

                            print(f"{dx}++{dy}")
                            flagxy=uart.car_move_xy_cm(dx,dy,15,15)
                            flagcount=0
                            if flagxy:
                                while True:
                                    _1,_2=uart.wait_for_data_packet(3)
                                    if _1:
                                        break
                                    else:
                                        flagxy=uart.car_move_xy_cm(dx,dy,15,15)
                                        flagcount+=1
                                        if flagcount==6:
                                            break
                                break
                            time.sleep(0.5)


