import time

import serial


class Uart:
    def __init__(self, com="/dev/ttyUSB0"):
        # 实例化串口对象
        self.uart = serial.Serial(com, 9600, timeout=0.01)
        self.param11=0
        self.param22=0
        self.param33=0
        self.param44=0
        self.param55=0
        self.param66=0
        if self.uart.isOpen():
            print("uart is ready")

    def close(self):
        self.uart.close()

    def wait_for_data_packet(self, timeout=10):
        time_begin = time.time()
        time_end = time_begin + timeout
        task = None
        color = None

        while time.time() < time_end:
            if self.uart.inWaiting() >= 4:
                packet = self.uart.read(4)
                print(packet)
                if packet[0] == 0xFF and packet[-1] == 0xFE:
                    task = packet[1]  # 提取task数据
                    color = packet[2]  # 提取color数据

                    print(f"task data: {task}")
                    print(f"color data: {color}")

                    break  # 数据已接收，退出循环
            #else:
                #time.sleep(0.02)
        #print("into2")
        if task is None and color is None:
            print("Timeout waiting for data packet.")
            return None, None  # 超时返回None, None
        else:
            #print(f"{task}  and  {color}")
            return task, color  # 返回接收到的task和color数据

    def uart_send_order(self, param1, param2,param3, param4,param5, param6, wait=True, timeout=10):
        myinput=bytes([0xFF,param1, param2,param3, param4,param5, param6,0x01])
        self.param11, self.param22,self.param33, self.param44,self.param55, self.param66=param1, param2,param3, param4,param5, param6
        self.uart.write(myinput)
        time.sleep(0.1)  # 避免单片机清零不及时
    def uart_send_yes(self):
        myinput=bytes([0xFF,self.param11, self.param22,self.param33, self.param44,self.param55, self.param66,0x01])
        self.uart.write(myinput)
        time.sleep(0.1)  # 避免单片机清零不及时


    def uart_send_command(self,  param1,param2 ,param3,param4,fn=0x01):
        print(f"@!{param1}|{param2}##{param3}|{param4}")
        myinput=bytes([0xbb,param1,param2,param3,param4,fn])
        self.uart.write(myinput)
        time.sleep(0.2)  # 避免单片机清零不及时
        # if wait is True:
        #     task, color=self.wait_for_32_ack(timeout)


    def car_move_xy_cm(self, x, y,dx=8,dy=8,wait=True):
         print(f"car X{x} Y{y}")
         x=int(0.5*x)###################################把比例砍了
         y=int(0.5*y)
         xabs=0x00
         yabs=0x00
         if(x<0):
             xabs=0x01
             x=-x
         if(y<0):
             yabs=0x01
             y=-y
         if(y>255):
             y=255
         if(x>255):
             x=255
         #y=20
         if(y<dx and x<dy):
             self.uart_send_command(param1=x, param2=y,param3=xabs,param4=yabs,fn=0x03)
             time.sleep(0.1)##############################################################################3
             #self.uart_send_command(param1=x, param2=y,param3=xabs,param4=yabs,fn=0x03)
             #time.sleep(0.1)
             #self.uart_send_command(param1=x, param2=y,param3=xabs,param4=yabs,fn=0x03)
             
             print("fnok")
             return True
             
         else:
             self.uart_send_command(param1=x, param2=y,param3=xabs,param4=yabs)
             return False
         

if __name__ == "__main__":
    uart=Uart()
    while True:
        uart.uart_send_yes()

    # myinput = bytes([0xF0, 0x03, 0xE0])
    # # uart.uart.write(myinput)
    # # uart.uart_send_order(3,2,1,3,2,1)
    # # time.sleep(1)
    # uart.car_move_xy_cm(-23,-46)
    # data,color=uart.wait_for_data_packet()