import utils
import cv2
import numpy as np 

widthTop = 141
heightTop = 67
widthBottom = 60
heightBttom = 162
wT = 480
hT = 240

curveList = []
avgVal = 10


#Check the main it self
def getLaneCurve(img, display=2):

    imgResult = img.copy()

    #Thresholding - Step 1
    imgThresh = utils.thresholding(img)
    
    #Warp - Step 2
    hTT, wTT, c =img.shape
    points =  np.float32([(widthTop, heightTop), (wT-widthTop,heightTop),
    (widthBottom, heightBttom), (wT-widthBottom,heightBttom)])
    imgWarp = utils.warpImage(imgThresh,points,wTT,hTT)
    #cv2.imshow('Warp',imgWarp)
    #cv2.imshow('Threshold',imgThresh)
    
    #Step 3
    middlePoints,imageHist = utils.getHistogram(imgWarp,display=True,minPer=0.5,region=4)
    curveAvaragePoints,imageHist = utils.getHistogram(imgWarp,display=True,minPer=0.9)
    #Curve Value
    print(curveAvaragePoints-middlePoints)
    curveRaw = curveAvaragePoints -middlePoints

    #Step 4
    curveList.append(curveRaw)
    if len(curveList)>avgVal:
        curveList.pop(0)
    curve = int(sum(curveList)/len(curveList))

    #Step 5 - Display
    if display != 0:
        imgInvWarp = utils.warpImage(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        #cv2.putText(imgResult, 'FPS ' + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 50, 50), 3);
    if display == 2:
        imgStacked = utils.stackImages(0.7, ([img, imgWarp,imgResult],
                                             [imageHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)
    

    #cv2.imshow('Histogram',imageHist)



    return None


if __name__=='__main__':
    #cap = cv2.VideoCapture('slownewvideo.mp4')
    cap = cv2.VideoCapture(0)
    
    while True:
        success, img = cap.read()
        img = cv2.resize(img,(480,240))
        getLaneCurve(img,display=2)
        #cv2.imshow('Vid',img)
        cv2.waitKey(1)