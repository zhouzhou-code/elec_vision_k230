import time,utime, os, sys,math
from media.sensor import *  # 导入sensor模块，使用摄像头相关接口
from media.display import * # 导入display模块，使用display相关接口
from media.media import *   # 导入media模块，使用meida相关接口
from machine import FPIOA   # 导入FPIOA模块，使用IO口相关接口
from machine import Pin,PWM,UART # 导入Pin、PWM和UART模块，使用GPIO、PWM和串口相关接口


def find_blob_center(threshold):
    blobs =img.find_blobs(threshold, roi=(0, 0, 640, 640), x_stride=2, y_stride=2, pixels_threshold=3000, margin=True)
    if blobs:
        b=blobs[0]
        cx= b.cx()
        cy= b.cy()
        return cx, cy
    return None, None


"""将四个点按顺时针顺序排序（左上开始）"""
def sort_corners_clockwise(points):
    if len(points) != 4:
        return points
    # 计算中心点
    center_x = sum(p[0] for p in points) / 4
    center_y = sum(p[1] for p in points) / 4

    # 计算极角并排序
    angles = []
    for p in points:
        dx = p[0] - center_x
        dy = p[1] - center_y
        angles.append(math.atan2(dy, dx))

    # 逆时针排序后反转得到顺时针
    sorted_points = [p for _, p in sorted(zip(angles, points), key=lambda x: x[0], reverse=False)]

    # 重新排列起点为左上附近
    x_sorted = sorted(sorted_points, key=lambda p: p[0])
    left_points = x_sorted[:2]
    left_sorted = sorted(left_points, key=lambda p: p[1])

    start_index = sorted_points.index(left_sorted[0])
    return sorted_points[start_index:] + sorted_points[:start_index]

def draw_rectangle(img,_corner_list, color=(0, 255, 0),draw_circle_f=True, thickness=2):
    # 绘制矩形边
    for i in range(4):
        start_point = _corner_list[i]
        end_point = _corner_list[(i + 1) % 4]  # 自动连接首尾点
        img.draw_line(start_point[0], start_point[1],end_point[0], end_point[1],color, thickness)
    # 在四个角点绘制小圆标识
    if draw_circle_f ==True:
        for (x, y) in _corner_list:
            img.draw_circle(x, y, 5, (0, 0, 255), thickness,fill=False)

#打包K230发送给单片机的数据
def pack_data(rect_coords, laser_point, problem_id):
    header = 0xAA
    footer = 0xFE
    buffer = bytearray([header, problem_id])
    # 4个角点,小端
    for x, y in rect_coords:
        buffer.append(x & 0xFF)
        buffer.append((x >> 8) & 0xFF)
        buffer.append(y & 0xFF)
        buffer.append((y >> 8) & 0xFF)
    # 激光坐标,小端序
    lx, ly = laser_point
    buffer.append(lx & 0xFF)
    buffer.append((lx >> 8) & 0xFF)
    buffer.append(ly & 0xFF)
    buffer.append((ly >> 8) & 0xFF)
    buffer.append(footer)
    return buffer


class KeyState:
    def __init__(self, key, debounce_ms=50):
        self.key = key
        self.last = 1  # 默认按键释放状态
        self.press_time = 0
        self.short_pressed = False
        self.long_pressed = False
        self.long_reported = False  # 长按已报告标志
        self.debounce_time = 0
        self.debounce_ms = debounce_ms  # 消抖时间(毫秒)

    def update(self):
        now = self.key.value()
        current_time = utime.ticks_ms()
        # 消抖处理
        if utime.ticks_diff(current_time, self.debounce_time) < self.debounce_ms:
            return
        # 按键按下事件 (下降沿)
        if self.last == 1 and now == 0:
            self.press_time = current_time
            self.long_reported = False  # 重置长按报告标志
            self.debounce_time = current_time
        # 按键释放事件 (上升沿)
        elif self.last == 0 and now == 1:
            press_duration = utime.ticks_diff(current_time, self.press_time)
            # 短按判定 (300ms内释放)
            if press_duration < 500:
                self.short_pressed = True
            self.debounce_time = current_time
        # 按键持续按下状态
        elif self.last == 0 and now == 0:
            press_duration = utime.ticks_diff(current_time, self.press_time)
            # 长按判定 (800ms后)
            if not self.long_reported and press_duration > 800:
                self.long_pressed = True
                self.long_reported = True  # 防止重复触发
        self.last = now

    def get_short_press(self):
        if self.short_pressed:
            self.short_pressed = False
            return True
        return False

    def get_long_press(self):
        if self.long_pressed:
            self.long_pressed = False
            return True
        return False

fpioa = FPIOA()
# 为IO分配相应的硬件功能
fpioa.set_function(59, FPIOA.GPIO59)
fpioa.set_function(61, FPIOA.GPIO61)
fpioa.set_function(60, FPIOA.PWM0)
fpioa.set_function(34, FPIOA.GPIO34) #key0
fpioa.set_function(35, FPIOA.GPIO35) #key1
fpioa.set_function(0, FPIOA.GPIO0)   #key2
fpioa.set_function(40,FPIOA.UART1_TXD)
fpioa.set_function(41,FPIOA.UART1_RXD)

ledb = Pin(59, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
ledr = Pin(61, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
beep = PWM(0, 4000, duty=50, enable=False)
key0 = Pin(34, Pin.IN, pull=Pin.PULL_UP, drive=7)
key1 = Pin(35, Pin.IN, pull=Pin.PULL_UP, drive=7)
key2 = Pin(0, Pin.IN, pull=Pin.PULL_DOWN, drive=7)
uart1 = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
uart2 = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

keys = [key0, key1, key2]
key_states = [KeyState(k) for k in keys]

adjust_threshold_mode = False # 调整阈值模式开关，K0管理该模式
threshold_idx = 0 #当前选中的阈值索引
recoder_edge_mode =False #记录铅笔框矩形角点模式，K1管理该模式
edge_corner_idx = 0 #当前选中的铅笔框角点索引
threshold_list =[30000,   #找矩形阈值
                80,240,   #灰度二值化阈值
                5000,     #曝光值
                1,        #roi区域缩放因子
                9, 100,  #Lmin, Lmax
                15, 98,   #Amin, Amax
                -13, 77,    #Bmin, Bmax 红色激光  (-128,127)
                1         #题号
                ]

LCD_width = 640
LCD_height = 480

roi_w = LCD_width // 2
roi_h = LCD_height // 2
roi_x = (LCD_width - roi_w) // 2
roi_y = (LCD_height - roi_h) // 2

default_roi = (roi_x, roi_y, roi_w, roi_h)
default_roi_corners = [
    (roi_x, roi_y),
    (roi_x + roi_w, roi_y),
    (roi_x + roi_w, roi_y + roi_h),
    (roi_x, roi_y + roi_h)
]
roi_center = (LCD_width // 2, LCD_height // 2)
new_roi = default_roi  # 初始时使用默认ROI
new_roi_corners = default_roi_corners  # 初始时使用默认ROI角点坐标

rect_coords=[] #发给单片机的矩形黑框角点坐标,顺时针
edge_coords=[(0,0),(0,0),(0,0),(0,0)] #发给单片机的边界矩形坐标,顺时针
edge_coords_ready = False #边界矩形坐标是否准备好
problem_id = 1 # 题号
default_red_thres_list=(9, 100,  #Lmin, Lmax
                       15, 98,   #Amin, Amax
                       -13, 77)   #Bmin, Bmax
red_coords = [] #红色激光坐标
# x' = c_x + (x - c_x) * s
# y' = c_y + (y - c_y) * s

try:
    sensor = Sensor(width=640, height=480) # 构建摄像头对象
    sensor.reset() # 复位和初始化摄像头
    sensor.set_framesize(width=LCD_width, height=LCD_height)   # 设置帧大小QVGA(320x240)，默认通道0
    sensor.set_pixformat(Sensor.RGB565) # 设置输出图像格式，默认通道0
    # 初始化LCD显示器，同时IDE缓冲区输出图像,显示的数据来自于sensor通道0。
    Display.init(Display.ST7701, width=LCD_width, height=LCD_height,to_ide=True)
    MediaManager.init() # 初始化media资源管理器
    sensor.run()        # 启动sensor
    clock = time.clock() # 构造clock对象

    while True:
        # print("hello")
        os.exitpoint() # 检测IDE中断
        clock.tick()   # 记录开始时间（ms）

        for ks in key_states: # 更新按键状态
            ks.update()
        """K230接收协议解析"""
        # bytes = uart1.read(3)
        # if bytes :
        #     if bytes[0] == 0xFF and bytes[2] == 0xFE:
        #         problem_id = bytes[1]
        #         print('收到题号:', problem_id)


        if key_states[0].get_long_press(): #全局检测K0是否长按，切换调阈值模式
            if(adjust_threshold_mode == False):
                adjust_threshold_mode = True
            elif(adjust_threshold_mode == True):
                adjust_threshold_mode = False
        if key_states[1].get_long_press(): #全局检测K1是否长按，切换记录铅笔框坐标模式
            if(recoder_edge_mode == False):
                recoder_edge_mode = True
            elif(recoder_edge_mode == True):
                recoder_edge_mode = False

        img = sensor.snapshot()

        if adjust_threshold_mode: # 如果处于调整阈值模式
            step=[1000,5,5,500,0.1,5,5,5,5,5,5,1] #变量步进值
            limit=[(0,30000),(0,255),(0,255),(0,10000),(0.5,2),(0,100),(0,100),(-128,127),(-128,127),(-128,127),(-128,127),(0,3)]
            if key_states[0].get_short_press(): #k0切索引
                threshold_idx = (threshold_idx + 1) % len(threshold_list)
            if key_states[1].get_short_press(): #k1加变量值
                threshold_list[threshold_idx]+=step[threshold_idx]
            if key_states[2].get_short_press(): #k2减变量值
                threshold_list[threshold_idx]-=step[threshold_idx]
            #变量值限幅
            for i in range(len(threshold_list)):
                if limit[i] is  None:
                    continue
                min_v, max_v = limit[i]
                threshold_list[i] = max(min_v, min(threshold_list[i], max_v))

            # img = sensor.snapshot() # 从通道0捕获一张图
            img_rect = img.to_grayscale(copy=True)
            img_rect = img_rect.binary([(threshold_list[1], threshold_list[2])])
            #找矩形画矩形
            rects = img_rect.find_rects(threshold=threshold_list[0],roi=new_roi)
            if not rects == None:
                for rect in rects:
                    corner = rect.corners()
                    sort_corners_clockwise(corner)
                    draw_rectangle(img_rect,corner)
            #找红激光，画十字
            red_blobs = img.find_blobs([default_red_thres_list],False,roi=new_roi, x_stride=2, y_stride=2,pixels_threshold=1,merge=True,margin=True)
            if red_blobs:
                max_red = max(red_blobs, key=lambda b: (b.area(), -b.y()))
                red_coords = (max_red.cx(), max_red.cy())
                img_rect.draw_cross(red_coords[0], red_coords[1], (0, 255, 255),size=25, thickness=3)
                print(red_coords)
            #题号赋值
            problem_id = threshold_list[11]

            if threshold_idx==0:
                img_rect.draw_string_advanced(10, 20,20, "rect: {}".format(threshold_list[0]), color=(255, 0, 0))
            elif threshold_idx==1:
                img_rect.draw_string_advanced(10, 20,20, "rec_Lmin: {}".format(threshold_list[1]), color=(255, 0, 0))
            elif threshold_idx==2:
                img_rect.draw_string_advanced(10, 20,20, "rec_Lmax: {}".format(threshold_list[2]), color=(255, 0, 0))
            elif threshold_idx==3:
                img_rect.draw_string_advanced(10, 20,20, "expouse: {}".format(threshold_list[3]), color=(255, 0, 0))
            elif threshold_idx==4:
                img_rect.draw_string_advanced(10, 20,20, "s_roi: {}".format(threshold_list[4]), color=(255, 0, 0))
            elif threshold_idx==5:
                img_rect.draw_string_advanced(10, 20,20, "red_Lmin: {}".format(threshold_list[5]), color=(255, 0, 0))
            elif threshold_idx==6:
                img_rect.draw_string_advanced(10, 20,20, "red_Lmax: {}".format(threshold_list[6]), color=(255, 0, 0))
            elif threshold_idx==7:
                img_rect.draw_string_advanced(10, 20,20, "red_Amin: {}".format(threshold_list[7]), color=(255, 0, 0))
            elif threshold_idx==8:
                img_rect.draw_string_advanced(10, 20,20, "red_Amax: {}".format(threshold_list[8]), color=(255, 0, 0))
            elif threshold_idx==9:
                img_rect.draw_string_advanced(10, 20,20, "red_Bmin: {}".format(threshold_list[9]), color=(255, 0, 0))
            elif threshold_idx==10:
                img_rect.draw_string_advanced(10, 20,20, "red_Bmax: {}".format(threshold_list[10]), color=(255, 0, 0))
            elif threshold_idx==11:
                img_rect.draw_string_advanced(10, 20,20, "ques_id: {}".format(threshold_list[11]), color=(255, 0, 0))

            #缩放roi区域
            s = threshold_list[4]
            c_x, c_y = roi_center
            new_roi_corners = [
                (int(c_x + (x - c_x) * s), int(c_y + (y - c_y) * s))
                for (x, y) in default_roi_corners
            ]
            # 计算新的roi区域（左上角x, 左上角y, 宽, 高）
            xs = [pt[0] for pt in new_roi_corners]
            ys = [pt[1] for pt in new_roi_corners]
            new_roi = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))

            draw_rectangle(img_rect,new_roi_corners,color=(255,0,0),draw_circle_f=False,thickness=2) #roi区域
            img_rect.compressed_for_ide()
            Display.show_image(img_rect)
            print(threshold_list)
        elif recoder_edge_mode:   # 如果处于记录铅笔框坐标模式
            #通过红色激光坐标，记录铅笔框角点坐标
            red_blobs = img.find_blobs([default_red_thres_list],False,roi=new_roi, x_stride=2, y_stride=2,pixels_threshold=1,merge=True,margin=True)
            if red_blobs:
                max_red = max(red_blobs, key=lambda b: (b.area(), -b.y()))
                red_coords = (max_red.cx(), max_red.cy())
                img.draw_cross(red_coords[0], red_coords[1], (0, 255, 0),size=10, thickness=2)
                print(red_coords)
            #按键切换索引，确定坐标
            if key_states[0].get_short_press(): #k0切索引
                edge_corner_idx = (edge_corner_idx + 1) % 4
            if key_states[1].get_short_press(): #k1确定该坐标
                edge_coords[edge_corner_idx] = red_coords
            if key_states[2].get_short_press(): #k2确定全部坐标
                edge_coords_ready = True
                print("k2按下",edge_coords_ready)
            img.draw_string_advanced(0, 10, 30, "point[{}]={}".format(edge_corner_idx,edge_coords[edge_corner_idx]), color=(255, 0, 0))
            img.draw_string_advanced(0, 40, 30, "ready_flag:{}".format(edge_coords_ready), color=(255, 0, 0))
            img.draw_string_advanced(0, 70, 30, "laser={}".format(red_coords), color=(255, 0, 0))
            img.compressed_for_ide()
            Display.show_image(img)
            print("edge_coords:", edge_coords, "edge_corner_idx:", edge_corner_idx)
        else:  #题目主逻辑
            if problem_id==3 or problem_id==4 :
                #找矩形
                img_rect = img.to_grayscale(copy=True)
                img_rect = img_rect.binary([(threshold_list[1], threshold_list[2])])
                rects = img_rect.find_rects(threshold=threshold_list[0],roi=new_roi)
                rects_sorted = sorted(rects, key=lambda r: (r.rect()[2] * r.rect()[3]), reverse=True) #按照面积从大到小排
                if len(rects_sorted) >= 2:
                    #处理外边框
                    outer = rects_sorted[0]
                    outer_ordered = sort_corners_clockwise(outer.corners())
                    # 处理内边框
                    inner = rects_sorted[1]
                    inner_ordered = sort_corners_clockwise(inner.corners())
                    #绘制矩形框
                    draw_rectangle(img,outer_ordered)
                    draw_rectangle(img,inner_ordered)
                    # 计算平均坐标
                    averaged_points = []
                    for i in range(4):
                        outer_p = outer_ordered[i]
                        inner_p = inner_ordered[i]
                        avg_x = (outer_p[0] + inner_p[0]) // 2
                        avg_y = (outer_p[1] + inner_p[1]) // 2
                        averaged_points.append((avg_x, avg_y))
                    rect_coords = averaged_points

                    print("外矩形:{}".format(outer_ordered))
                    print("内矩形:{}".format(inner_ordered))
                    print("平均坐标:{}".format(rect_coords))
                #找红激光，画十字
                red_blobs = img.find_blobs([default_red_thres_list],False,roi=new_roi, x_stride=2, y_stride=2,pixels_threshold=1,merge=True,margin=True)
                if red_blobs:
                    max_red = max(red_blobs, key=lambda b: (b.area(), -b.y()))
                    red_coords = (max_red.cx(), max_red.cy())
                    print(red_coords)
                    img.draw_cross(red_coords[0], red_coords[1], (0, 255, 0),size=20, thickness=3)

                draw_rectangle(img,new_roi_corners,color=(255,255,255),draw_circle_f=False,thickness=2) #roi区域
                img.draw_string_advanced(0, 10, 25, "fps: {}".format(clock.fps()), color=(255, 0, 0))
                img.compressed_for_ide()
                Display.show_image(img)
                #打包发送
                if rect_coords and red_coords:
                    buf=pack_data(rect_coords, red_coords, problem_id=3) #打包数据
                    uart1.write(buf)

            elif problem_id==1 or problem_id==2:
                #寻找色块，参数依次为颜色阈值, 是否反转，roi, x_stride, y_stride, pixels_threshold, margin(是否合并)
                red_blobs = img.find_blobs([default_red_thres_list],False,roi=new_roi, x_stride=2, y_stride=2,pixels_threshold=1,merge=True,margin=True)
                if red_blobs:
                    max_red = max(red_blobs, key=lambda b: (b.area(), -b.y()))
                    red_coords = (max_red.cx(), max_red.cy())
                    img.draw_cross(red_coords[0], red_coords[1], (0, 255, 0),size=20, thickness=3)
#                    print(red_coords)
                img.draw_string_advanced(0, 10, 25, "fps: {}".format(clock.fps()), color=(255, 0, 0))
                img.compressed_for_ide()
                Display.show_image(img)
                print("edge_coords:", edge_coords, "laser:",red_coords,"flag:",edge_coords_ready)
                #打包发送
                if edge_coords and red_coords and edge_coords_ready:
                    buf=pack_data(edge_coords, red_coords, problem_id=1) #打包数据
                    uart1.write(buf)
                    print("buf:",buf)

        # img.compressed_for_ide()
        # Display.show_image(img)

# IDE中断释放资源代码
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    # sensor stop run
    if isinstance(sensor, Sensor):
        sensor.stop()
    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()
