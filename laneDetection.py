import cv2 as cv 
import numpy as np


def isolate_lanes(img):
    height=gray_blurred.shape[0]
    width=int(gray_blurred.shape[1]/2)
    pts=[(width-1000, height-100), (width-210, height-850), (width+10, height-850), (width+700, height-100)]
    mask=np.zeros_like(img)
    cv.fillPoly(mask, np.array([pts]),255)
    canny=cv.bitwise_and(img,mask)
    return canny

def slope(setone,settwo):
    xdif=settwo[0]-setone[0]
    ydif=settwo[1]-setone[1]
    if ydif == 0: 
        return 0
    return (xdif/ydif)

def intercept(slope, pair):
    x, y = pair[0]
    intercept = (y/x)-slope
    return intercept

def average_coordinate_pairs(slopes):
    running_p1xsum=0
    running_p2xsum=0
    running_p1ysum=0
    running_p2ysum=0
    for slope in slopes:
        running_p1xsum+=slope[0][0]
        running_p1ysum+=slope[0][1]
        running_p2xsum+=slope[1][0]
        running_p2ysum+=slope[1][1]
    if len(slopes) == 0:
        return ((0,0),(0,0))
    average_p1x=int(running_p1xsum/len(slopes))
    average_p2x=int(running_p2xsum/len(slopes))
    average_p1y=int(running_p1ysum/len(slopes))
    average_p2y=int(running_p2ysum/len(slopes))
    p1=(average_p1x, average_p1y)
    p2=(average_p2x,average_p2y)
    set_of_points=(p1,p2)
    return set_of_points

def make_lines(pair):
    x_coords=[pair[0][0], pair[1][0]]
    y_coords=[pair[0][1], pair[1][1]]
    if x_coords[0] != 0: 
        f_slope, f_intercept=np.polyfit(x_coords, y_coords, 1)
        y1=1800-100
        y2=int(y1*3/5)
        x1=int((y1-f_intercept)/f_slope)
        x2=int((y2-f_intercept)/f_slope)
        return np.array([x1,y1,x2,y2])
        

    return None

def find_midpoint(set1, set2):
    set1=list(set1)
    set2=list(set2)
    x_average=int((set1[0]+set2[0])/2)
    y_average=int((set1[1]+set2[1])/2)
    return (x_average, y_average)
cap=cv.VideoCapture('testVideo.mp4')

while True: 
    isTrue, frame = cap.read()
    frame_to_gray=cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray_blurred=cv.GaussianBlur(frame_to_gray, (7,7), 0)
    gray_canny=cv.Canny(gray_blurred, 20,50)
    canny=isolate_lanes(gray_canny)
    colored_canny=cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    positive_slopes=[]
    negative_slopes=[]

    canny_mask=np.zeros_like(canny)
    lines = cv.HoughLines(canny, 1, np.pi/180, 50, None, 0, 0)
    if lines is not None:
        for line in lines: 
            rho=line[0][0]
            theta=line[0][1]
            a=np.cos(theta)
            b=np.sin(theta)
            x0=a*rho
            y0=b*rho
            pt1 = (int(x0 + 3000*(-b)), int(y0 + 3000*(a)))
            pt2 = (int(x0 - 3000*(-b)), int(y0 - 3000*(a)))

        
            if slope(pt1,pt2) > 0: 
                positive_slopes.append([pt1,pt2])
            else: 
                negative_slopes.append([pt1,pt2])
        
        positive_line = make_lines(average_coordinate_pairs(positive_slopes))
        negative_line = make_lines(average_coordinate_pairs(negative_slopes))
        positive_set1=(positive_line[0],positive_line[1])
        positive_set2=(positive_line[2],positive_line[3])
        if negative_line is None: 
            negative_line=[0,0,0,0]
        negative_set1=(negative_line[0],negative_line[1])
        negative_set2=(negative_line[2],negative_line[3])
        cv.line(frame, positive_set1, positive_set2, (255,255,0), 5, cv.LINE_AA)
        cv.line(frame, negative_set1, negative_set2, (255,255,0), 5, cv.LINE_AA)
        positive_midpoint=find_midpoint(positive_set1, positive_set2)
        negative_midpoint=find_midpoint(negative_set1, negative_set2)
        lane_midpoint=find_midpoint(positive_midpoint, negative_midpoint)
        cv.circle(frame, positive_midpoint,15, (0,255,0), cv.FILLED)
        cv.circle(frame, negative_midpoint,15,(0,255,0), cv.FILLED)
        cv.circle(frame, lane_midpoint, 15, (0,255,0), cv.FILLED)
    cv.imshow("Canny", canny)
    cv.imshow("Frame", frame)
    if cv.waitKey(1) & 0xFF==ord('q'):
        break

cap.release()
cv.destroyAllWindows()
