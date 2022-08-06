import cv2
import numpy as np

def thresholding(img):
    imageHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([0,0,190])
    upperWhite = np.array([91,16,255])
    maskWhite = cv2.inRange(imageHSV,lowerWhite,upperWhite)
    return maskWhite

def warpImage(img,points,w,h, inv = False):
    #Point 1, Then add width and height to it
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    
    #transformation Matrix
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2,pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1,pts2)

    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp

def getHistogram(img,minPer=0.1,display = False, region=1):
    if region == 1:
        histValues = np.sum(img,axis=0) # Sum of the first collumn
    else:
        histValues = np.sum(img[img.shape[0]//region:,:],axis=0)
    #print(histValues)
    maxValues = np.max(histValues)
    #print(maxValues)
    #Thresholding
    minValue = minPer*maxValues

    indexArrays = np.where(histValues >= minValue)
    basePoints = np.average(indexArrays)
    #print(basePoints)

    #Plotting Histogram and Basepoints 
    if display:
        imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x,intensity in enumerate(histValues):
            cv2.line(imgHist,(x,img.shape[0]),(x,img.shape[0]-intensity//255//region),(255,0,255),1)
            cv2.circle(imgHist,(int(basePoints),img.shape[0]),20,(0,255,255),cv2.FILLED)
        return basePoints,imgHist
    return basePoints


def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver