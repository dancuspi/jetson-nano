from threading import Thread
import cv2
import time
import numpy as np
import jetson.inference
import jetson.utils
from datetime import datetime
import sys
import RPi.GPIO as GPIO
import sys, tty, termios, time
import select
import os
import socket
import requests 
from Queue import Queue


class vStream:
    def __init__(self, src, width, height):
        self.width = width
        self.height = height
        self.capture = cv2.VideoCapture(src)
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
    	reboot = 0
        while True:
        	#print("Trying Thread")
        	try:
		    _, self.frame = self.capture.read()
		    self.frame2 = cv2.resize(self.frame, (self.width, self.height))
		    
		except:
			pass
			#print("Cam not available")
			time.sleep(1)
			
    			break
			
    def getFrame(self):
        return self.frame2

def InternetC():
	while True:
		time.sleep(1)
		try:
			requests.get('https://www.google.com/').status_code
			#print("Internet is active")
			global reboot
			reboot = 0
		except:
			global reboot
			reboot = 1
			pass

font = cv2.FONT_HERSHEY_SIMPLEX
startTime = time.time()

alert = 0
reboot = 0

dispW = 640
dispH = 480

image_width = 640
image_height = 480
rtsp_latency = 0

width = 1535
height = 767
dtav = 0
scaleFactor = 0.8

net = jetson.inference.detectNet("ped-100", threshold=0.5)
# net=jetson.inference.imageNet('googleNet')
uri1 = "rtsp://admin:password@192.168.1.1:554/cam/realmonitor?channel=2&subtype=0"
uri2 = "rtsp://admin:password@192.168.1.1:554/cam/realmonitor?channel=6&subtype=0"

uri3 = "rtsp://admin:password@192.168.1.1:554/cam/realmonitor?channel=8&subtype=0"
uri4 = "rtsp://admin:password@192.168.1.1:554/cam/realmonitor?channel=5&subtype=0"

uri5 = "rtsp://admin:password@192.168.1.1:554/cam/realmonitor?channel=3&subtype=0"
uri6 = "rtsp://admin:password@192.168.1.1:554/cam/realmonitor?channel=4&subtype=0"
camSet1 = (
    "rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false"
).format(uri1, rtsp_latency, image_width, image_height)
camSet2 = (
    "rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false"
).format(uri2, rtsp_latency, image_width, image_height)
camSet3 = (
    "rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false"
).format(uri3, rtsp_latency, image_width, image_height)
camSet4 = (
    "rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false"
).format(uri4, rtsp_latency, image_width, image_height)
camSet5 = (
    "rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false"
).format(uri5, rtsp_latency, image_width, image_height)
camSet6 = (
    "rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false"
).format(uri6, rtsp_latency, image_width, image_height)

cam1 = vStream(camSet1, dispW, dispH)
cam2 = vStream(camSet2, dispW, dispH)
cam3 = vStream(camSet3, dispW, dispH)
cam4 = vStream(camSet4, dispW, dispH)
cam5 = vStream(camSet5, dispW, dispH)
cam6 = vStream(camSet6, dispW, dispH)
        

ICThread=Thread(target=InternetC)
ICThread.daemon = True
ICThread.start()

while True:
	#print("Trying Detection")
	try:
		
		myFrame1 = cam1.getFrame()
		myFrame2 = cam2.getFrame()
		myFrame3 = cam3.getFrame()
		myFrame4 = cam4.getFrame()
		myFrame5 = cam5.getFrame()
		myFrame6 = cam6.getFrame()

		VFrame1 = np.vstack((myFrame1, myFrame2))
		VFrame2 = np.vstack((myFrame3, myFrame4))
		VFrame3 = np.vstack((myFrame5, myFrame6))
		FFrame = np.hstack((VFrame1, VFrame2, VFrame3))
		FFrameSmall = cv2.resize(FFrame, (0, 0), fx=scaleFactor, fy=scaleFactor)
		
		frame = cv2.cvtColor(FFrameSmall, cv2.COLOR_BGR2RGBA)
		frame = jetson.utils.cudaFromNumpy(frame)
		# cv2.imshow('ComboCam',Frame)
		detections = net.Detect(frame, width, height)
		now = datetime.now()
		
		if not detections:
		 	alert=0
		else :
			for detect in detections:
		   
			    
			    ID = detect.ClassID
			    top = int(detect.Top)
			    left = int(detect.Left)
			    bottom = int(detect.Bottom)
			    right = int(detect.Right)
			    
			    item = net.GetClassDesc(ID)
			    
			    # print(item,top,left,bottom,right)
			    if item == "person":
				if alert == 0:
					print("Intruso Detectado",now.strftime("%d-%m-%Y %H:%M:%S"),str(round(fps, 1)),)
					alert=1
			    	
			    cv2.rectangle(FFrameSmall, (left, top), (right, bottom), (0, 255, 0), 1)
			    cv2.putText(FFrameSmall, item, (left, top + 20), font, 0.5, (0, 0, 255), 1)
		
		dt = time.time() - startTime
		startTime = time.time()
		dtav = 0.9 * dtav + 0.1 * dt
		fps = 1 / dtav
		# cv2.rectangle(FFrameSmall, (0, 0), (140, 40), (0, 0, 255), -1)
		# cv2.putText(FFrameSmall,str(round(fps, 1)) + " fps",(0, 25),font,0.75,(0, 255, 255),2,)
		Titulo = "Seguridad" + str(round(fps, 1)) + " fps"
		cv2.imshow("", FFrameSmall)
		cv2.setWindowTitle("", Titulo)
		# cv2.moveWindow('ComboCam',0,0)
		#print (reboot)	
	except:
		pass
		print("frame not available")

	"""if reboot == 1:
		
		cam1.capture.release()
		cam2.capture.release()
		cam3.capture.release()
		cam4.capture.release()
		cam5.capture.release()
		cam6.capture.release()
		cv2.destroyAllWindows()
		time.sleep(2)
		while True:
			print("Desconectado")
			try:
				if reboot == 0:
					print("Conectado")
					time.sleep(5)
					print("Internet disconnected")
					print("Saliendo")
					
					os.system("python Dual-Camera-T.py")
					print "Restarting..."
					exit()
					break
				else:
					print("Desconectado",now.strftime("%d-%m-%Y %H:%M:%S"))
					time.sleep(1)
			except:
				pass
					
	"""
		
	if cv2.waitKey(1) == ord("q"):
		while True:
		    	print("Saliendo")
			cam1.capture.release()
			cam2.capture.release()
			cam3.capture.release()
			cam4.capture.release()
			cam5.capture.release()
			cam6.capture.release()
			cv2.destroyAllWindows()
			exit(1)
			break
     
