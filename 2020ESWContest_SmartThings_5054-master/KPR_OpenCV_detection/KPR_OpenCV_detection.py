import cv2
import math
import numpy as np
import RPi.GPIO as GPIO
import time

line_width = 40


GPIO.setmode(GPIO.BCM)

GPIO.setup(18, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(23, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(24, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(17, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(27, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(22, GPIO.OUT,initial=GPIO.LOW)

PWMA=GPIO.PWM(17,1000) #GPIO.PWM(GPIO PIN,Duty Rate)
PWMA.start(30)
PWMB=GPIO.PWM(18,1000)
PWMB.start(30)

def Motor_drive_RL(direction_r,pwm_r,direction_l,pwm_l) :
    if direction_l == 1:
       #print('Left motor Forward')
       PWMB.ChangeDutyCycle(pwm_l)
       GPIO.output(23, GPIO.HIGH)
       GPIO.output(24, GPIO.LOW)  
    elif direction_l == 0:
       #print('Left motor Stop')
       PWMB.ChangeDutyCycle(pwm_l)
       GPIO.output(23, GPIO.LOW)
       GPIO.output(24, GPIO.LOW)
    elif direction_l == -1:
       #print('Left motor Backward')
       PWMB.ChangeDutyCycle(pwm_l)
       GPIO.output(23, GPIO.LOW)
       GPIO.output(24, GPIO.HIGH)   
       
    if direction_r == 1:
       #print('Right motor Forward')    
       PWMA.ChangeDutyCycle(pwm_r)
       GPIO.output(27, GPIO.LOW)
       GPIO.output(22, GPIO.HIGH)
    elif direction_r == 0:
       #print('Right motor Stop')        
       PWMA.ChangeDutyCycle(pwm_r)
       GPIO.output(27, GPIO.LOW)
       GPIO.output(22, GPIO.LOW)
    elif direction_r == -1:
       #print('Right motor left')        
       PWMA.ChangeDutyCycle(pwm_r)
       GPIO.output(27, GPIO.HIGH)
       GPIO.output(22, GPIO.LOW)

def region_of_interest1(image): 
    height = image.shape[0]
    width  = image.shape[1] 
    polygons = np.array([[(0, 350+10),(50,350-10), (width-40,350-10),(width, 350+10)]]) 
    mask = np.zeros_like(image)       
    # Fill poly-function deals with multiple polygon 
    cv2.fillPoly(mask, polygons, 255)        
    # Bitwise operation between canny image and mask image 
    masked_image = cv2.bitwise_and(image, mask)  
    return masked_image 


def find_lane_center(n, stat, centroid):    
    lane_location = []
    for i in range(1, n):
        area = stat[i, cv2.CC_STAT_AREA]
        width = stat[i, cv2.CC_STAT_WIDTH]
        if(area>300 and width < line_width+10 and width > line_width-10) :
            cx = int(centroid[i,0])
            cy = 320
            lane_location.append(cx)
            print('lane width %3d'%(width))
    
    lane_location.sort()
    return lane_location

def draw_lane_center(image,lane):
    for i in range(0,len(lane)):        
        cx = lane[i]
        cy = 320
        cv2.rectangle(image, (cx-15, cy-20), (cx+15, cy+20), (255,0,255), 2)  
    return image   
    
    
def find_lane_slope(lines):
    slope_data = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            if y1 != y2 :
                parameters = np.polyfit((y1, y2), (x1, x2), 1)
                slope = parameters[0]
                intercept = parameters[1]
                slope_data.append(slope) 
            
                
        
        slope_avg = sum(slope_data) / len(slope_data)
        slope_degree = -math.atan(slope_avg)*180/3.14159
        #print(slope_degree)
        return slope_degree

def canny_edge_detector(image): 
      
    # Convert the image color to grayscale 
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  
      
    # Reduce noise from the image 
    blur = cv2.GaussianBlur(gray_image, (5, 5), 0)  
    canny = cv2.Canny(blur, 40, 130) 
    return canny 

def threshold_image(image, threshold_value,color):
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  
    if color == 1 :  # white line detection
        ret,img_result = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)
    else :
        ret,img_result = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY_INV)
    return img_result

def region_of_interest(image): 
    height = image.shape[0]
    width  = image.shape[1] 
    polygons = np.array([[(0, height-60),(50, 280), (width-40, 280),(width, height-60)]]) 
    mask = np.zeros_like(image)       
    # Fill poly-function deals with multiple polygon 
    cv2.fillPoly(mask, polygons, 255)        
    # Bitwise operation between canny image and mask image 
    masked_image = cv2.bitwise_and(image, mask)  
    return masked_image 

def draw_hough_line(image, lines):
    image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)  
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(image_color,(x1,y1),(x2,y2),(0,0,255),1)
    return image_color


capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = capture.read()
    canny_edge = canny_edge_detector(frame)
    
    
    canny_edge = region_of_interest(canny_edge)
    lines = cv2.HoughLinesP(canny_edge, 2, np.pi / 180, 20,np.array([]),20,10)
    steering_angle1 = find_lane_slope(lines)
    
    threshold_img = threshold_image(frame,70,0)
    threshold_img = region_of_interest1(threshold_img)
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(threshold_img)
    
    canny_edge_color = draw_hough_line(canny_edge, lines)
    lane_center_pos = find_lane_center(n, stats,centroids)
    canny_edge_color = draw_lane_center(canny_edge_color,lane_center_pos)
        
  
    print('Lane Angle %3d'%(steering_angle1))
    left_pwm  = 20 + 0.3*steering_angle1
    right_pwm = 20 - 0.3*steering_angle1
    if left_pwm < 5 :
        left_pwm = 5
    if right_pwm < 5 :
        right_pwm = 5
    print('pwm r : %3d | pwm_l  " %3d'%(right_pwm,left_pwm))
        
    Motor_drive_RL(1,right_pwm,1,left_pwm)    
        
    cv2.namedWindow('Camera Window')
    cv2.moveWindow('Camera Window',20,0)
    cv2.imshow('Camera Window', frame)
    
    
    cv2.namedWindow('Edge Window')
    cv2.moveWindow('Edge Window',720,0)
    cv2.imshow('Edge Window', canny_edge)
    
    cv2.namedWindow('Line Window')
    cv2.moveWindow('Line Window',20,530)
    cv2.imshow('Line Window',canny_edge_color )
    
    if cv2.waitKey(1) > 0: break

   
GPIO.cleanup()
capture.release()
cv2.destroyAllWindows()

