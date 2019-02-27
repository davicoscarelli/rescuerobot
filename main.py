from picamera.array import PiRGBArray
from picamera import PiCamera
from Tkinter import *
import Tkinter as Tk
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO



def nothing(x):
                # any operation
        pass

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT)
GPIO.output(40, GPIO.HIGH)


#Camera setup

camera = PiCamera()
camera.resolution = (640, 360)
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 360))
time.sleep(0.1)



cv2.namedWindow("Calibration")
cv2.createTrackbar("A", "Block Calibration", 0, 255, nothing)
cv2.createTrackbar("B", "Block Calibration", 0, 255, nothing)
cv2.createTrackbar("C", "Block Calibration", 0, 255, nothing)

for frame in camera.capture_continuous(rawCapture, format="bgr",
use_video_port=True):


#Setup
        key = cv2.waitKey(1) & 0xFF     

        GreenL = False
        GreenR = False

        image = frame.array     
        roi = image[200:250, 0:439]
        R = image[220:260, 320:640]
        L = image[220:260, 0:320]

        Blackline = cv2.inRange(roi, (0,0,0), (60,60,60))

        GreenboxR = cv2.inRange(R, (0,65,0), (100,200,100))
        GreenboxL = cv2.inRange(L, (0,65,0), (100,200,100))

        kernel = np.ones((3,3), np.uint8)

        Blackline = cv2.erode(Blackline, kernel, iterations=5)
        Blackline = cv2.dilate(Blackline, kernel, iterations=9) 

        GreenboxR = cv2.erode(GreenboxR, kernel, iterations=5)
        GreenboxR = cv2.dilate(GreenboxR, kernel, iterations=9)
        GreenboxL = cv2.erode(GreenboxL, kernel, iterations=5)
        GreenboxL = cv2.dilate(GreenboxL, kernel, iterations=9) 

        img_grnR = cv2.findContours(GreenboxR.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours_grnR, hierarchy_grnR =
cv2.findContours(GreenboxR.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        img_grnL = cv2.findContours(GreenboxL.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours_grnL, hierarchy_grnL =
cv2.findContours(GreenboxL.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        img_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours_blk, hierarchy_blk =
cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)




#Black line detection

        if len(contours_blk) > 0:       
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, y_min), (w_min, h_min), ang = blackbox

                setpoint = 320
                error = int(x_min - setpoint)

                cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1,
(255, 0, 0), 2)

#Crossroadsdetection

        if str(contours_blk[0][0][0][0]) == "1":
                cv2.putText(image,"Encrusilhada",(200, 220),
cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 0, 255), 2)
        else:
                cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)

#R Green box detection

        if len(contours_grnR) > 0:
                GreenR= True
                x_grnR, y_grnR, w_grnR, h_grnR = cv2.boundingRect(contours_grnR[0])
                centerx_grnR = x_grnR + (w_grnR/2)
                cv2.line(R,(centerx_grnR, 200), (centerx_grnR, 250), (0,255,0),3)

                if len(contours_grnL) > 0:
                        cv2.putText(image,"Beco sem Saida",(50, 180),
cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                else:
                        cv2.putText(image,"Direita",(400, 200), cv2.FONT_HERSHEY_SIMPLEX,
1, (0, 255, 0), 2)

#L Green box detection

        if len(contours_grnL) > 0:
                GreenL= True
                x_grnL, y_grnL, w_grnL, h_grnL = cv2.boundingRect(contours_grnL[0])
                centerx_grnL = x_grnL + (w_grnL/2)
                cv2.line(L,(centerx_grnL, 200), (centerx_grnL, 250), (0,255,0),3)

                if len(contours_grnR) > 0:
                        cv2.putText(image,"Beco sem Saida",(50, 180),
cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                else:
                        cv2.putText(image,"Esquerda",(50, 180), cv2.FONT_HERSHEY_SIMPLEX,
1, (0, 255, 0), 2)

#Block Detection

        if key == ord("o"):

                camera.resolution = (3280, 2464)
                camera.rotation = 0
                camera.start_preview(fullscreen=False, window = (100, 20, 1024, 768))
                time.sleep(5)
                camera.capture('image.jpg')
                camera.stop_preview()
                image = cv2.imread('image.jpg',0)
                imagem = 'oq01.jpg'
                Achou = False
                Rampa = False

                for i in range(1, 5):

                        template = cv2.imread(imagem,0)
                        w, h = template.shape[::-1]
                        res = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
                        threshold = 0.75
                        loc = np.where(res>=threshold)

                        for pt in zip(*loc[::-1]):
                                if imagem == 'oq04.jpg':
                                        Rampa = True
                                else:
                                        Achou = True
                                if Achou:
                                        print("Obstaculo")
                                        print("Tentativa: %d" %(i))
                                        o = 0
                                        camera.resolution = (640, 360)
                                        break
                                else:
                                        imagem = ('oq0%d.jpg' %(i+1))
                                if Rampa:
                                        print("Rampa!!!")
                                        o = 0
                                        camera.resolution = (640, 360)
                                        break
                        if Achou:
                                break
                        elif Rampa:
                                break
                        camera.resolution = (640, 360)
                if Rampa == False and Achou == False:
                        print("Nada encontrado")



#Visualizer

        A = cv2.getTrackbarPos("A", "Calibration")
        B = cv2.getTrackbarPos("B", "Calibration")
        C = cv2.getTrackbarPos("C", "Calibration")
        cv2.imshow("Line Follower - OBR 2019", image)
        cv2.imshow("Blackline", Blackline)
        cv2.imshow("GreenboxR", GreenboxR)
        cv2.imshow("GreenboxL", GreenboxL)      
        rawCapture.truncate(0)  

        if key == ord("q"):
                break

GPIO.output(40, GPIO.LOW)
