import cv2
from Uart_GXS import *
from Cartest1 import *
from thread2 import *

conf1=0.85 #夹物料置信度
diff1=30#夹取物料像素差

conf2=0.80 #圆环置信度
diff2=8#圆环像素差

conf3=0.90 #放置物料置信度
diff3=15#放置物料像素差




if __name__ == "__main__":
    #初始化
    flagcam1 = False
    flagcam2 = False
    rknn1 = rknn("wk.rknn", 0.6, 0.6)
    rknn2 = rknn("rknn_sz_new.rknn", 0.6, 0.6)
    uart = Uart()
    width, height = 640, 480
    cam1 = cv2.VideoCapture(2)
    cam1.set(3, width)
    cam1.set(4, height)

    if cam1.isOpened():
        flagcam1 = True
        print("cam1 ready")
    else:
        print("cam1 can't open")
    if flagcam1:
        while True:
            _, img = cam1.read()
            order1, order2 = qr_scan(img)

            if order1 is not None:
                print(order1)
                print(order2)
                uart.uart_send_order(order1[0], order1[1], order1[2], order2[0], order2[1], order2[2])
                break
            cv2.imshow("img", img)
            cv2.waitKey(1)


    #启用线程
    camera = Camera(camera_id=0)
    while True:
        frame = camera.get_latest_frame()
        if frame is not None:
            print("camera ready!")
            break  # 没有这个窗口会来不及渲染，cv2.waitKey() 返回-1；如果有按键按下，则返回按键的ASCII码值
        else:
            print("camera none ready")

    while True:
        task, color = uart.wait_for_data_packet()
        #task,color=1,1
        print(task)
        print("++")
        print(color)
        if task == 1:
            preX, preY = 0, 0
            while True:
                #with lock:
                frame = camera.get_latest_frame()
                cv2.imshow("1",frame)
                print("start")
                boxs, scores, classes = rknn1.rknn_detect(frame)

                if scores is not None:
                    if scores > conf1 and classes == color:
                        top, left, right, bottom = boxs
                        top = int(top)
                        left = int(left)
                        right = int(right)
                        bottom = int(bottom)
                        centerx = int((left + right) / 2)
                        centery = int((top + bottom) / 2)
                        print(f"pre+ X:{preX},Y:{preY}")
                        print(f"now+ X:{centerx},Y:{centery}")
                        preX = centerx
                        preY = centery

                        if (abs(centerx - preX) + abs(centery - preY)) < diff1:
                            print("yes")
                            uart.uart_send_yes()
                            break
                        time.sleep(0.5)

        if task == 2:
            if color == 1:
                color = 2
            elif color == 2:
                color = 1
            while True:
                frame = camera.get_latest_frame()
                boxs, scores, classes = rknn2.rknn_detect(frame, color)
                if scores is not None:
                    if scores > conf2 and classes == color:
                        left, top, right, bottom = boxs
                        top = int(top)
                        left = int(left)
                        right = int(right)
                        bottom = int(bottom)
                        centerx = (float)(left + right) / 2
                        centery = (float)(top + bottom) / 2
                        dx = centerx - 320
                        dy = centery - 320
                        print(f"center{centerx}++{centery}")
                        print(f"top{top} left{left} right{right} bottem{bottom}")

                        print(f"{dx}++{dy}")
                        flagxy = uart.car_move_xy_cm(dx, dy)
                        flagcount = 0
                        if flagxy:
                            while True:
                                _1, _2 = uart.wait_for_data_packet(1)
                                if _1:
                                    break
                                else:
                                    flagxy = uart.car_move_xy_cm(dx, dy,diff2,diff2)
                                    flagcount += 1
                                    if flagcount == 6:
                                        break
                            break
                        time.sleep(0.5)

        if task == 3:
            while True:
                frame = camera.get_latest_frame()
                boxs, scores, classes = rknn1.rknn_detect(frame, color)
                if scores is not None:
                    if scores > conf3 and classes == color:
                        left, top, right, bottom = boxs
                        top = int(top)
                        left = int(left)
                        right = int(right)
                        bottom = int(bottom)
                        centerx = (float)(left + right) / 2
                        centery = (float)(top + bottom) / 2
                        dx = centerx - 320
                        dy = centery - 320
                        print(f"center{centerx}++{centery}")
                        print(f"top{top} left{left} right{right} bottem{bottom}")
                        print(f"{dx}++{dy}")
                        flagxy = uart.car_move_xy_cm(dx, dy, diff3, diff3)
                        flagcount = 0
                        if flagxy:
                            while True:
                                _1, _2 = uart.wait_for_data_packet(3)
                                if _1:
                                    break
                                else:
                                    flagxy = uart.car_move_xy_cm(dx, dy, 15, 15)
                                    flagcount += 1
                                    if flagcount == 6:
                                        break
                            break
                        time.sleep(0.5)


