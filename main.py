import sensor, utime, image, time, pyb, math #import librares

EXPOSURE_TIME_SCALE = 0.8


yellow_threshold = [(36, 70, -1, 127, 3, 127)]
blue_threshold = [(13, 90, -105, 27, -128, -25)]
red_threshold = [(13, 95, 23, 113, -2, 107)]

callibrate_center = False

center1 = [160, 121]
center2 = [158, 112]

white1 = (-2.5, -7, -1.47639)
white2 = (64, 60, 63)

robot = 1

if robot == 1:
    center = center1
    white = white1
    EXPOSURE_TIME_SCALE = 0.8
    img_radius = 145
    robot_radius = 22
else:
    center = center2
    white = white2
    EXPOSURE_TIME_SCALE = 0.7
    img_radius = 135
    robot_radius = 11
uart = pyb.UART(3, 230400, timeout = 100, timeout_char = 100)
uart.init(230400, bits=8, parity=False, stop=1, timeout_char=100) #initialize UART

led3 = pyb.LED(2)

################################### setup camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
current_exposure_time_in_microseconds =  sensor.get_exposure_us()
sensor.set_auto_exposure(True, \
    exposure_us = int(current_exposure_time_in_microseconds* EXPOSURE_TIME_SCALE))
clock = time.clock()
sensor.skip_frames(time = 500)


sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
print(sensor.get_rgb_gain_db())
sensor.set_auto_whitebal(False, white)#(-2.5, -7, -1.47639)
sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds =  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* EXPOSURE_TIME_SCALE))
clock = time.clock()
sensor.skip_frames(time = 1000) #delay

image_height = sensor.height() #get image framesize
image_width = sensor.width()

yellow_x = 0
yellow_y = 0
blue_x = 0
blue_y = 0

yellow = [0]*6
blue = [0]*6
ball = [0] * 6
old_area = 0

old_i = 0

ball_angle = 0
ball_distance = 50

yellow_angle = 0
yellow_distance = 100
blue_angle = 180
blue_distance = 80

yellow_is_see = False
blue_is_see = False
            #sm   pxl
distance = [
            [210, 144],
            [140, 128],
            [130, 126],
            [120, 123],
            [110, 120],
            [100, 117],
            [90, 114],
            [80, 109],
            [75, 107],
            [70, 106],
            [65, 103],
            [60, 99],
            [55, 93],
            [50, 89],
            [45, 78],
            [40, 77],
            [35, 70],
            [30, 65],
            [25, 57],
            [20, 52],
            [15, 39],
            [10, 27],
            [5, 23]
                    ]

clock = time.clock()

a = 0

def back():
    if yellow[5] > center[1] :
       return 'y'
    else:
       return 'b'

# function that calculates distance to the gates
def get_distance(x, y):
    number = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    return number

def linearize(num):
    num = abs(num)
    old_i = 0
    i = 0
    old_err = 1000
    for mas in distance: #getting data from mass
        err = abs(num) - abs(mas[1]) #calculating error from input data and the second element of arr
        if abs(old_err) > abs(err): #finding the nearest element to the input data
            i += 1
        else:
            break
        old_err = err
        old_i = i
    result = 0
    dist1 = 0
    dist2 = 0
    if i > 1 and i < len(distance) and (num != distance[i - 1][1]):
        if abs(num) > abs(distance[i - 1][1]):
            dist1 = distance[i - 1][1] - num
            dist2 = distance[i][1] - num
            result = distance[i - 1][0] + (distance[i - 1][0] - distance[i][0]) / (dist1 + dist2) * dist1
        else:
            dist1 = num - distance[i][1]
            dist2 = 1
            result = distance[i][0] + (distance[i - 1][0] - distance[i][0]) / (dist1 + dist2) * dist1
    else:
        result = distance[i - 1][0]
    return result

def crc8(data, len): #function that calculates check sum
    crc = 0xFF
    j = 0
    for i in range(0, len):
        crc = crc ^ data[i];
        for j in range(0, 8):
            if (crc & 0x80):
                crc = (crc << 1) ^ 0x31
            else:
             crc = crc << 1
    return crc

data = bytearray(7)

def send_data(num1, num2, num3, num4, num5, num6):
    uart.writechar(255)
    num1 = int((num1) / 4)
    num2 = int((num2) / 4)
    num3 = int((num3) / 4)
    num4 = int((num4) / 4)
    num5 = int((num5) / 4)
    num6 = int((num6) / 4)

    if num1 + 127 > 253:
       data[0] = 253
    elif num1 + 127 < 0:
       data[0] = 0
    else:
       data[0] = num1

    if num2 + 127 > 253:
       data[1] = 253
    elif num2 + 127 < 0:
       data[1] = 0
    else:
       data[1] = num2

    if num3 + 127 > 253:
       data[2] = 253
    elif num3 + 127 < 0:
       data[2] = 0
    else:
       data[2] = num3

    if num4 + 127 > 253:
       data[3] = 253
    elif num4 + 127 < 0:
       data[3] = 0
    else:
       data[3] = num4

    if num5 + 127 > 253:
       data[4] = 253
    elif num5 + 127 < 0:
       data[4] = 0
    else:
       data[4] = num5

    if num6 + 127 > 253:
       data[5] = 253
    elif num6 + 127 < 0:
       data[5] = 0
    else:
       data[5] = num6
    data[6] = crc8(data, 6)

    uart.writechar(int(data[0]))
    uart.writechar(int(data[1]))
    uart.writechar(int(data[2]))
    uart.writechar(int(data[3]))
    uart.writechar(int(data[4]))
    uart.writechar(int(data[5]))
    uart.writechar(int(data[6]))



while(True):
    clock.tick()
    #print(utime.ticks_ms())
    if callibrate_center == False:
        img = sensor.snapshot().mask_circle(center[0], center[1], img_radius)#.binary(green_threshold, zero=True)#.binary(black_threshold, zero=True) #get corrected image
        img.draw_circle(center[0], center[1], robot_radius, (0, 0, 0), fill = True)
        if robot == 1:
            img.draw_rectangle(142, 97, 36, 7, (0, 0, 0), 1, True)
            img.draw_line(147, 95, 173, 95, (0, 0, 0), 7)
            img.draw_line(143, 140, 171, 145, (0, 0, 0), 7)
            img.draw_line(141, 106, 137, 115, (0, 0, 0), 7)
            img.draw_line(143, 137, 138, 127, (0, 0, 0), 7)
            img.draw_line(154, 146, 164, 148, (0, 0, 0), 7)
            img.draw_line(148, 103, 143, 107, (0, 0, 0), 7)
            img.draw_line(171, 142, 177, 147, (0, 0, 0), 7)
            img.draw_line(178, 101, 187, 121, (0, 0, 0), 7)
            img.draw_line(144, 145, 152, 145, (0, 0, 0), 5)
            img.draw_line(136, 132, 139, 139, (0, 0, 0), 5)
        else:
            img.draw_line(145, 121, 153, 126, (0, 0, 0), 5)
            img.draw_line(153, 126, 164, 125, (0, 0, 0), 5)
    else:
        img = sensor.snapshot()
    old_area = 0



    yellow_is_see = False
    blue_is_see = False

    #detecting yellow gate
    for blob in img.find_blobs(yellow_threshold, pixels_threshold=50, area_threshold=200, merge=True, margin = 20):#finding gates
        if(blob[2] * blob[3] > old_area):
            old_area = blob[2] * blob[3]
            img.draw_rectangle(blob[0], blob[1], blob[2], blob[3], (200, 200, 0), 2)
            #yellow = [blob[0], blob[1], blob[0] + blob[2], blob[1] + blob[3], blob.cx(), blob.cy()]
            yellow = [blob[0], blob[1], blob[0] + blob[2], blob[1] + blob[3],(blob[0] + int(blob[2] / 2)), (blob[1] + int(blob[3] / 2))]
            yellow_x = -(yellow[4] - center[0])
            yellow_y = -(yellow[5] - center[1])
            yellow_distance = linearize(get_distance(yellow_x, yellow_y))
            yellow_angle = math.floor(math.atan2(yellow_x, yellow_y) * 57.3) - 90
            if yellow_angle == 0:
                yellow_angle += 1

    if(yellow_angle > 360):
        yellow_angle -= 360
    elif(yellow_angle < 0):
        yellow_angle += 360

    if(old_area < 50):
        yellow_distance = 0
    old_area = 0

    #detecting blue gate
    for blob in img.find_blobs(blue_threshold, pixels_threshold=50, area_threshold=50, merge=True, margin = 10): #finding gates
        if(blob[2] * blob[3] > old_area and blob[2] * blob[3] > 60):
            old_area = blob[2] * blob[3]
            img.draw_rectangle(blob[0], blob[1], blob[2], blob[3], (0, 0, 200), 2)
            #blue = [blob[0], blob[1], blob[0] + blob[2], blob[1] + blob[3], blob.cx(), blob.cy()]
            blue = [blob[0], blob[1], blob[0] + blob[2], blob[1] + blob[3],(blob[0] + int(blob[2] / 2)), (blob[1] + int(blob[3] / 2))]
            blue_x = -(blue[4] - center[0])
            blue_y = -(blue[5] - center[1])
            blue_distance = linearize(get_distance(blue_x, blue_y))
            blue_angle = math.floor(math.atan2(blue_x, blue_y) * 57.3) - 90
            if(blue_angle > 360):
                blue_angle -= 360
            elif(blue_angle < 0):
                blue_angle += 360
    if(old_area < 50):
        blue_distance = 0
    old_area = 0

    ball_distance = 0
    #detecting ball
    for blob in img.find_blobs(red_threshold, pixels_threshold = 5, area_threshold = 5, merge=True, margin = 2): #finding gates
        if(blob[2] * blob[3] > old_area and blob[2] * blob[3] > 5 and blob[2] * blob[3] < 400):
            old_area = blob[2] * blob[3]
            img.draw_rectangle(blob[0]- 5, blob[1] - 5, blob[2] + 10, blob[3] + 10, (0, 0, 0), 1)
            #blue = [blob[0], blob[1], blob[0] + blob[2], blob[1] + blob[3], blob.cx(), blob.cy()]
            ball = [blob[0], blob[1], blob[0] + blob[2], blob[1] + blob[3],(blob[0] + int(blob[2] / 2)), (blob[1] + int(blob[3] / 2))]
            ball_x = -(ball[4] - center[0])
            ball_y = -(ball[5] - center[1])
            ball_distance = linearize(get_distance(ball_x, ball_y))
            ball_angle = math.floor(math.atan2(ball_x, ball_y) * 57.3) - 92
            if(ball_angle > 360):
                ball_angle -= 360
            elif(ball_angle < 0):
                ball_angle += 360
    old_area = 0

    #img.draw_line(yellow[4], yellow[5], blue[4], blue[5], (255, 255, 255), 2) #line between centers of gates


    if yellow_angle > 359:
        yellow_angle -= 360
    if blue_angle > 359:
        blue_angle -= 360
    if ball_angle > 359:
        ball_angle -= 360
    #print(ball_angle)#start from yellow gate
    #send_data(10, 20, 30, 40, 50, 60)
    send_data(yellow_angle, yellow_distance, blue_angle, blue_distance, ball_angle, ball_distance)
    img.draw_circle(blue[4], blue[5], 3, (255, 255, 255))
    img.draw_circle(yellow[4], yellow[5], 3, (255, 255, 255))
    if callibrate_center == False:
        img.draw_circle(ball[4], ball[5], 3, (255, 255, 255))
    img.draw_circle(center[0], center[1], 3, (255, 255, 255))
