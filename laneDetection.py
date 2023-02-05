import cv2 as cv 
import numpy as np
from math import sqrt

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

def create_lane_line(lane):
    return None


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

def find_midpoint(set1, set2):
    set1=list(set1)
    set2=list(set2)
    x_average=(set1[0]+set2[0])/2
    y_average=(set1[1]+set2[1])/2
    midpoint=[x_average,y_average]
    return midpoint

cap=cv.VideoCapture('testVideo.mp4')

while True: 
    isTrue, frame = cap.read()
    frame_to_gray=cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray_blurred=cv.GaussianBlur(frame_to_gray, (7,7), 0)
    gray_canny=cv.Canny(gray_blurred, 20,60)
    canny=isolate_lanes(gray_canny)
    colored_canny=cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    positive_slopes=[]
    negative_slopes=[]

    canny_mask=np.zeros_like(canny)
    lines = cv.HoughLinesP(canny, 1, np.pi/180, 50, None, 50, 10)
    if lines is not None:
        for line in lines: 
            x1, y1, x2, y2 = line[0]
            if slope((x1,y1), (x2,y2)) > 0:
                positive_slopes.append([(x1,y1), (x2,y2)])
            else:
                negative_slopes.append([(x1,y1), (x2,y2)])

            #cv.line(, (x1,y1), (x2,y2), (255,200,0),3, cv.LINE_AA)
        cv.line(frame,average_coordinate_pairs(positive_slopes)[0], average_coordinate_pairs(positive_slopes)[1], (255,150,0),5, cv.LINE_AA)
        cv.line(frame,average_coordinate_pairs(negative_slopes)[0], average_coordinate_pairs(negative_slopes)[1], (255,150,0),5, cv.LINE_AA)

        
    cv.imshow("Canny", frame)
    if cv.waitKey(1) & 0xFF==ord('q'):
        break

cap.release()
cv.destroyAllWindows()
