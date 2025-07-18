import cv2
import numpy as np
import time
import math
# use these for get image
seeThres = True;
pipThres = False;

pipMorph = False
seeMorph = False
#testing, delete later
# red = False;
# blue = False;
# yellow = True
y1 = 4+1/32 #mm
c = {"x":54.5, "y":42} # deg, deg
a = 25 #deg
b= {"x":640, "y":480} # px, px
h= y1/math.cos(math.radians(90-a)) #mm/untyped = mm
l = h*math.tan(math.radians(c["y"]/2)) # mm/untyped = mm

def heightFilter():
    def filter(contour):
        x,y,w,h = cv2.boundingRect(contour)
        return y>300 #TUNE THIS!
    return filter

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    ##record time
    ##testing, delete later
    # llrobot = [1.0 if red else 0.0, 1.0 if blue else 0.0, 1.0 if yellow else 0.0, 0.0,0.0,0.0,0.0,0.0]


    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (0,0,0),(-1,0,0));
    ####tune these
    ##red
    if(llrobot[0]==1):
        img_threshold = cv2.bitwise_or(
            cv2.inRange(img_hsv, (170, 120, 80), (180, 255, 255)),
            cv2.inRange(img_hsv, (0, 120, 80), (0, 255, 255))
        )
    ##Blue
    if(llrobot[1]==1):
        img_threshold = cv2.bitwise_or(
            cv2.inRange(img_hsv, (90, 125, 0), (140, 255, 200)),
            img_threshold
        )
    ##yellow
    if(llrobot[2]==1):
        img_threshold = cv2.bitwise_or(
            cv2.inRange(img_hsv, (12, 180, 200), (35, 255, 255)),
            img_threshold
        )
    #

    ###MORPHONONONGONONSOCMLKSHY
    #define morphos
    dilate1 = cv2.getStructuringElement(cv2.MORPH_RECT,(10,10))
    erode = cv2.getStructuringElement(cv2.MORPH_RECT,(15,15))
    dilate2 = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

    ##create morphs
    morpho = cv2.dilate(img_threshold,dilate1)
    morpho = cv2.erode(morpho, erode)
    morpho = cv2.dilate(morpho, dilate2)

    ###test canny and houghlines


    ###main things
    ##find contours
    contours, _ = cv2.findContours(morpho,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    llpython = []
    # contours = sort(contours);

    contours = np.array(filter(heightFilter(), contours)) # TEST THIS might work with list(stuff)

    largestContour = np.array([[]])
    if(len(contours)>0):
        largestContour = max(contours, key=cv2.contourArea)
        # print(largestContour);
        if(cv2.contourArea(largestContour)>1500):
            try:
                (x, y), (majA, minA), angle = cv2.fitEllipse(largestContour)
                cv2.putText(image,
                    "(x,y): "+str(int(x))+", "+str(int(y)),
                    (int(x)+35, int(y)-165),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.putText(image,
                    "(w,h): "+str(int(majA))+", "+str(int(minA)),
                    (int(x)+28, int(y)-130),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.putText(image,
                    "(a): "+str(int(angle)),
                    (int(x)+60, int(y)-95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)

                l1 = (y*l)/(b["x"]/2) # px*mm/px = mm
                a1 = math.degrees(math.atan(l1/h)) # deg
                a2 = (90-a)-a1 # deg
                horiz = math.tan(math.radians(a2))*y1 # mm

                l2 = math.tan(math.radians(c["x"]/2))*horiz
                l3 = -(l2*x)/(b["x"]/2)

                cv2.putText(image,
                    "(Δx,Δy): "+str(int(l3))+str(int(horiz)),
                    (int(x)+20, int(y)-60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)
            except:
                print("brok")
            else:
                llpython = [l3, horiz, x,y,majA,minA,angle];
        else:
            largestContour = np.array([[]])
            # print("smol")
    # else:
    #     print("none")

    ###Image rendering (for port)
    for contour in contours:
        ####tune sizes
        if(cv2.contourArea(contour)>4000):
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image,[box],0,(0,255,0),5)
        elif(cv2.contourArea(contour)>2000):
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image,[box],0,(255,0,0),2)


    ###debug views (for port)

    if(pipThres):
        image = img_threshold
    elif(pipMorph):
        image = morpho
    elif(seeThres):
        displayThres = cv2.addWeighted(
                    cv2.cvtColor(img_threshold,cv2.COLOR_GRAY2RGB), 0.25,
                    image, 0.8, 0.1)
        image = displayThres
    elif(seeMorph):
        displayMorph = cv2.addWeighted(
                    (cv2.cvtColor(morpho,cv2.COLOR_GRAY2RGB)), .5,
                    image,1,0)
        image = displayMorph

    # tbh, i only use llpython LOL
    return largestContour, image, llpython
