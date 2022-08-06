import cv2
import numpy as np 


def getLaneCurve(img):
    h, w, c =img.shape
    points = valTrackbars()
    imgWarp = warpImage(img,points,w,h)
    imageWarpPoints = drawPoints(img,points)
    cv2.imshow('Warp', imgWarp)
    cv2.imshow('Warp Points', imageWarpPoints)

def nothing(a):
    pass

def initializeTrackbars(initialTrackbarVals,wT=480,hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars",480,240)
    cv2.createTrackbar("Width Top","Trackbars", initialTrackbarVals[0],wT//2, nothing)
    cv2.createTrackbar("Height Top","Trackbars", initialTrackbarVals[1],hT, nothing)
    cv2.createTrackbar("Width Bottom","Trackbars", initialTrackbarVals[2],wT//2, nothing)
    cv2.createTrackbar("Height Bottom","Trackbars", initialTrackbarVals[3],hT, nothing)

def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBttom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop,heightTop),
    (widthBottom, heightBttom), (wT-widthBottom,heightBttom)])

    return points

def warpImage(img,points,w,h):
    #Point 1, Then add width and height to it
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    #transformation Matrix
    matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp

def drawPoints(img,points):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
    return img



if __name__=='__main__':
    #cap = cv2.VideoCapture('slownewvideo.mp4')
    cap = cv2.VideoCapture(0)
    initialTrackbarVals = [100,100,100,100]
    initializeTrackbars(initialTrackbarVals)
    while True:
        success, img = cap.read()
        img = cv2.resize(img,(480,240))
        getLaneCurve(img)
        #cv2.imshow('Vid',img)
        cv2.waitKey(1)