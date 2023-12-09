"""
References:
 - User raja_961, “Autonomous Lane-Keeping Car Using Raspberry
Pi and OpenCV”. Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/

 - Team There are four of us (Noemi Moreno, Ethan Peck, James Ding, Davis Jackson), "Autonomous RC Car". Hackster.
 URL: https://www.hackster.io/there-are-four-of-us/autonomous-rc-car-d71671
 
Github Repo:
 https://github.com/czhou101/lane_following_rc_car
"""

import cv2
import numpy as np
import math
import board
import busio
import adafruit_mcp4728
import time
import matplotlib.pyplot as plt

def detect_stop(frame):
    stop = 0
    hasStop = 0
    #detect red if it fits in lower and higher thresholds of read color on both ends of the color spectrum
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    red_mask = mask = mask0+mask1
    #cv2.imshow("red mask",red_mask)
    #if frame has at least 100 pixels of red then there must be a stop sign
    hasStop = np.sum(red_mask)
    print(hasStop)
    if hasStop > 5000000:
        stop = 1
        print("Stop sign detected!")
    return stop

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 50, 0], dtype = "uint8") # lower limit of blue color
    upper_blue = np.array([120, 255, 255], dtype="uint8") # upper limit of blue color
    mask = cv2.inRange(hsv,lower_blue,upper_blue) # this mask will filter out everything but blue

    # detect edges
    edges = cv2.Canny(mask, 60, 100) 
 #   cv2.imshow("edges",edges)
    return edges

def region_of_interest(edges):

    height, width = edges.shape # extract the height and width of the edges frame
    mask = np.zeros_like(edges) # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask) 
    cv2.imshow("roi",cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=1, maxLineGap=150)
    #print(len(line_segments))
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        print("no line segment detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3

    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                #print("skipping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane 
    # all coordinate points are in pixels
    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0: 
        slope = 0.1    

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6): # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  
    return line_image

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)  
    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90 

    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def compute_steering_deviation(frame, imshow=True):
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    if imshow:
        heading_image = display_heading_line(lane_lines_image, steering_angle)
        # cv2.imshow("heading line", heading_image)
    return steering_angle - 90

def check_for_stop_sign(frame):

    height, width = frame.shape[:2] # extract the height and width of the edges frame
    cropped_frame = frame[int(height * .75) : height, 0 : width]
    hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
    # Range of red in HSV
    lower_red = np.array([10,100,20], dtype="uint8")
    upper_red = np.array([25,255,255], dtype="uint8")
# Filter out non red pixels and get count of number of red pixels
    mask = cv2.inRange(hsv,lower_red, upper_red)
    #cv2.imshow("filter", mask)
    num_red_px = cv2.countNonZero(mask)
    #print(num_red_px)
    return num_red_px  > stop_threshold


#################need to add initializations#############
# connect i2c
i2c = busio.I2C(board.SCL, board.SDA)
# i2c address is 0x64
mcp4728 =  adafruit_mcp4728.MCP4728(i2c,0x64)
# DEFAULT NO MOVEMENT STATE - INITILIZATION
# c channel controls speed
mcp4728.channel_c.value = int(65535*311/660)
# b channel controls direction
mcp4728.channel_b.value = int(65535*23/33)
# delay
time.sleep(2)

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

def read_speed():
    file_path = f"/sys/module/spd_encoder_driver/parameters/speed"

    try:
        with open(file_path, "r") as file:
            value = int(file.read().strip())  # Convert the read string to an integer
            return value
    except FileNotFoundError:
        print(f"Error: Parameter '{parameter_name}' not found.")
        return None
    except Exception as e:
        print(f"Error reading parameter '{parameter_name}': {e}")
        return None

def plot_pd(p_vals, d_vals, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    if show_img:
        plt.show()
    plt.clf()


def plot_voltage(steer_voltage, speed_voltage, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(steer_voltage))
    ax1.plot(t_ax, steer_voltage, '-', label="steer_voltage")
    ax1.plot(t_ax, speed_voltage, '-', label="speed_voltage")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("Voltage (0-65535)")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("Voltage changes and error over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("voltage_plot.png")

    if show_img:
        plt.show()
    plt.clf()

# Variables for stop detection
stop_threshold = 500
stop_cnt = 0
stop = False
last_stop = 0
time_stop = 0
stop_time = 0
stop_flag = 0

#Variables to be updated each loop
lastTime = 0 
lastError = 0

neutral = int(65535*23/33)

#variables for plotting
p_vals = []
d_vals = []
error_vals = []
steer_voltage = []
speed_voltage = []
speeds = []

#variables for speed control
curr_encoder_time = 0
last_encoder_time = 0
speed_frac = 400/660

counter = 0
max_counter = 100
speed_neutral = int(65535*311/660)
speed_forward = int(65535*400/660)
mcp4728.channel_c.value = speed_forward#speed_neutral

# *************************** MAIN LOOP ****************************
while True:
    ret,frame = video.read()
    
    
    # ************************ SPEED CONTROL *****************************
    
    last_encoder_time = curr_encoder_time
    curr_encoder_time = read_speed()
    
    # Speed up or down based on whether encoder interval increased or decreased by a significant amt
    # vs last interval
    if(last_encoder_time - curr_encoder_time > 4000000):
        speed_frac = speed_frac * 1.01
        mcp4728.channel_c.value = int(65535 * speed_frac)
        print('accelerating')
    elif(last_encoder_time - curr_encoder_time < -4000000):
        speed_frac = speed_frac * .99
        mcp4728.channel_c.value = int(65535 * speed_frac)
        print('decelerating')

    # for plotting speed
    speed_voltage.append(int(65535*speed_frac))



    # ********************** STEERING CONTROL *************************
    # calculate error
    error  = compute_steering_deviation(frame)
    now = time.time() # current time variable
    dt = now - lastTime
    time_stop = now - last_stop
    # Choose left vs right proportional constant
    if error < 0:
       Kp = 1400
    else:
       Kp = 500
    # Derivative constant
    Kd = Kp*0.1
    
    # set PD
    derivative = Kd * (error - lastError) / dt
    proportional = Kp * error

    # for plotting
    p_vals.append(proportional)
    d_vals.append(derivative)
    error_vals.append(error)

    # Set cap and floor so we dont have too sharp turns
    PD = int(neutral + derivative + proportional)
    if PD < 25000:
        PD = 25000
    elif PD > 57000:
        PD = 57000

    # Output steering voltage to DAC
    mcp4728.channel_b.value = PD
    
    #for plotting
    steer_voltage.append(PD)

    
    #*************************** STOP DETECTION ******************************
    stop = check_for_stop_sign(frame)
    #if red pixels detected in bottom 1/4 of frame, set flag
    if stop:
        stop_flag = 1
    # start counting after flag set
    if stop_flag and stop_time == 0:
        stop_time = now
    # stop for 2 seconds, 1 second after stop first detected
    if stop_flag and stop_cnt == 0 and now - stop_time > 1:
        mcp4728.channel_c.value = speed_neutral
        time.sleep(2)
        stop_cnt += 1
        mcp4728.channel_c.value = speed_forward
        last_stop = time.time()
        stop_time = 0
        stop_flag = 0
    # exit after second stop
    elif stop_flag and stop_cnt == 1 and now - stop_time > 1:
        mcp4728.channel_c.value = speed_neutral
        mcp4728.channel_b.value = int(65535*23/33)
        break


    lastError = error
    lastTime = time.time()
    key = cv2.waitKey(1)
    
    # exit w/ esc
    if key == 27:
        mcp4728.channel_c.value = speed_neutral
        # b channel controls direction
        mcp4728.channel_b.value = int(65535*23/33)
        break

#    plot_pd(p_vals, d_vals, error_vals, True)


video.release()

# generate plots
plot_pd(p_vals, d_vals, error_vals, True)
plot_voltage(steer_voltage, speed_voltage, error_vals, True)
np.save( 'speeds', np.array(speeds))
