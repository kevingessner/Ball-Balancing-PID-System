import cv2
import numpy as np
import time
import imutils
import tkinter as tk
import tkinter.messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from math import *
import os
import sys


dataPath = os.path.join(os.path.dirname(sys.argv[0]), "data.txt")
print("Loading %s" % dataPath)
with open(dataPath, 'r') as f:
    lines = f.read().splitlines()
lines = lines[:-11]     #last 11 lines are comments
lines = lines[1:]       #first line is header

dataDict = {}

camHeight = 480
camWidth = 640
cam = cv2.VideoCapture(0)
cam.set(3,camWidth)
cam.set(4,camHeight)

getPixelColor = False
H,S,V = 0,0,0

mouseX,mouseY = 0,0

for i in range(0,len(lines)):
    key, value = lines[i].split("#")
    alpha, beta = key.split("|")
    angleA, angleB, angleC = value.split("|")
    dataDict[(float(alpha),float(beta))] = (float(angleA), float(angleB), float(angleC))

MODE_HOLD_ON_TARGET = 'hold'
MODE_CIRCLE = 'circle'
MODE_EIGHT = 'eight'


controllerWindow = tk.Tk()
controllerWindow.title("Control")
controllerWindow.geometry("820x500")
controllerWindow["bg"]="white"
controllerWindow.resizable(0, 0)

BallMovementMode = tk.StringVar()
BallMovementMode.set(MODE_HOLD_ON_TARGET)

videoWindow = tk.Toplevel(controllerWindow)
videoWindow.title("Preview")
videoWindow.resizable(0, 0)  # window will resize to the image size
lmain = tk.Label(videoWindow)
lmain.pack()
videoWindow.withdraw()

graphWindow = tk.Toplevel(controllerWindow)
graphWindow.title("Position history")
graphWindow.resizable(0, 0)
graphCanvas = tk.Canvas(graphWindow, width=camHeight+210, height=camHeight)
graphCanvas.pack()
graphWindow.withdraw()

pointsListCircle = []
def createPointsListCircle(radius):
    global pointsListCircle
    for angle in range(0,360):
        angle=angle-90
        pointsListCircle.append([radius*cos(radians(angle))+240,radius*sin(radians(angle))+240])
createPointsListCircle(150)

pointsListEight = []
def createPointsListEight(radius):
    global pointsListEight
    for angle in range(270,270+360):
        pointsListEight.append([radius*cos(radians(angle))+240,radius*sin(radians(angle))+240+radius])
    for angle in range(360,0,-1):
        angle=angle+90
        pointsListEight.append([radius*cos(radians(angle))+240,radius*sin(radians(angle))+240-radius])
createPointsListEight(80)

pointCounter = 0
def setTargetForMovement():
    global pointCounter, targetX, targetY
    if BallMovementMode.get() == MODE_CIRCLE:
        sliderCoefP.set(15)
        if pointCounter >= len(pointsListCircle):
            pointCounter = 0
        point = pointsListCircle[pointCounter]
        targetX, targetY = point[0], point[1]
        pointCounter += 7
    elif BallMovementMode.get() == MODE_EIGHT:
        sliderCoefP.set(15)
        if pointCounter >= len(pointsListEight):
            pointCounter = 0
        point = pointsListEight[pointCounter]
        targetX, targetY = point[0], point[1]
        pointCounter += 7


def setTargetWithMouse(mousePosition):
    global targetX, targetY
    if mousePosition.y > 10:
        refreshGraph()
        targetX,targetY = mousePosition.x,mousePosition.y


def getMouseClickPosition(mousePosition):
    global mouseX,mouseY
    global getPixelColor
    mouseX,mouseY = mousePosition.x,mousePosition.y
    getPixelColor = True

showVideoWindow = False
def toggleCameraFrameWindow():
    global showVideoWindow
    showVideoWindow = not showVideoWindow

showCalqueCalibrationBool = False
def showCalqueCalibration():
    global showCalqueCalibrationBool
    showCalqueCalibrationBool = not showCalqueCalibrationBool

showGraph = False
def toggleGraphWindow():
    global showGraph
    showGraph = not showGraph
graphWindow.protocol("WM_DELETE_WINDOW", toggleGraphWindow)


t = 480
targetY = 240
targetX = 240
def paintGraph():
    global t,targetY,x,y,prevX,prevY,alpha,prevAlpha
    global showGraphPositionX,showGraphPositionY, showGraphAlpha
    if showGraph:
        graphWindow.deiconify()
        BafficherGraph.configure(text="Hide graph")
        if showGraphPositionX.get() == 1:
            graphCanvas.create_line(t-3,prevX,t,x, fill="#b20000", width=2)
        if showGraphPositionY.get() == 1:
            graphCanvas.create_line(t-3,prevY,t,y, fill="#0069b5", width=2)
        if showGraphAlpha.get() == 1:
            graphCanvas.create_line(t-3,240-prevAlpha*3,t,240-alpha*3, fill="#8f0caf", width=2)
        if t >= 480:
            t = 0
            graphCanvas.delete("all")
            graphCanvas.create_line(3,3,480,3,fill="black", width=3)
            graphCanvas.create_line(3,480,480,480,fill="black", width=3)
            graphCanvas.create_line(3,3,3,480,fill="black", width=3)
            graphCanvas.create_line(480,3,480,480,fill="black", width=3)
            graphCanvas.create_line(550,32,740,32,fill="#b20000", width=5)
            graphCanvas.create_line(550,53,740,53,fill="#0069b5", width=5)
            graphCanvas.create_line(550,73,740,73,fill="#8f0caf", width=5)
            if showGraphPositionX.get() == 1:
                graphCanvas.create_line(3,targetX,480,targetX,fill="#ff7777", width=2)
            if showGraphPositionY.get() == 1:
                graphCanvas.create_line(3,targetY,480,targetY,fill="#6f91f7", width=2)
        t += 3
    else:
        graphWindow.withdraw()
        BafficherGraph.configure(text="Show graph")

def refreshGraph():
    global t
    t=480

def endProgam():
    controllerWindow.destroy()


sliderHDefault = 15
sliderSDefault = 70
sliderVDefault = 70
sliderCoefPDefault = 10
sliderCoefIDefault = 0.1
sliderCoefDDefault = 5.7

def resetSlider():
    sliderH.set(sliderHDefault)
    sliderS.set(sliderSDefault)
    sliderV.set(sliderVDefault)
    sliderCoefP.set(sliderCoefPDefault)
    sliderCoefI.set(sliderCoefIDefault)
    sliderCoefD.set(sliderCoefDDefault)

def donothing():
    pass

def lowerPlate():
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Alert", "Remember to remove the plate"):
            print("Lowering arms")
            ser.write(("descendreBras\n").encode())
    else:
        if tkinter.messagebox.askokcancel("Alert","Arduino is not connected"):
            donothing()


def raisePlate():
    global alpha
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Alert", "Remember to remove the plate"):
            print("Raising arms")
            ser.write((str(dataDict[(0,0)])+"\n").encode())
            alpha = 0
    else:
        if tkinter.messagebox.askokcancel("Alert","Arduino is not connected"):
            donothing()

def servosTest():
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Alert", "The plate must be in place"):
            for i in range(2):
                beta = 0
                alpha = 35
                while beta < 360:
                    ser.write((str(dataDict[(alpha,beta)])+"\n").encode())
                    ser.flush()
                    time.sleep(0.002)
                    beta = round(beta+0.2,2)
                    print(alpha,beta)
            time.sleep(1)
            ser.write((str(dataDict[(0,0)])+"\n").encode())
    else:
        if tkinter.messagebox.askokcancel("Alert", "Arduino is not connected"):
            donothing()


arduinoIsConnected = False
def connectArduino():
    global ser
    global label
    global arduinoIsConnected
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print("checking port %s" % p)
        if "Arduino" in p.description or 'FT232' in p.description:
            print('connecting port %s' % p)
            ser = serial.Serial(p[0], 19200, timeout=1)
            time.sleep(1) #give the connection a second to settle
            label.configure(text="Arduino connected", fg="#36db8b")
            arduinoIsConnected = True

startBalanceBall = False
def startBalance():
    global startBalanceBall
    if arduinoIsConnected == True:
        if startBalanceBall == False:
            startBalanceBall = True
            BStartBalance["text"] = "Stop"
        else:
            startBalanceBall = False
            BStartBalance["text"] = "Start"
    else:
        if tkinter.messagebox.askokcancel("Alert", "Arduino is not connected"):
            donothing()

sommeErreurX = 1
sommeErreurY = 1
timeInterval = 1
alpha, beta, prevAlpha, prevBeta = 0,0,0,0
omega = 0.2
def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, targetX, targetY):
    global omega
    global sommeErreurX, sommeErreurY
    global alpha, beta, prevAlpha, prevBeta
    global startBalanceBall, arduinoIsConnected
    
    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    Ix = Kp*(targetX-ballPosX) + Ki*sommeErreurX + Kd*((prevBallPosX-ballPosX)/0.0333)
    Iy = Kp*(targetY-ballPosY) + Ki*sommeErreurY + Kd*((prevBallPosY-ballPosY)/0.0333)
    
    Ix = round(Ix/10000, 4)
    Iy = round(Iy/10000, 4)
    
    if Ix == 0 and Iy == 0:
        alpha = 0
        beta = 0
    
    elif Ix != 0 and sqrt(Ix**2 + Iy**2) < 1:
        beta = atan(Iy/Ix)
        alpha = asin(sqrt(Ix**2 + Iy**2))
        beta = degrees(beta)
        alpha = degrees(alpha)
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) < 1:
        if Iy > 0:
            beta = 90
            alpha = asin(sqrt(Ix**2 + Iy**2))
        elif Iy < 0:
            beta = 270
            alpha = asin(sqrt(Ix**2 + Iy**2))
        alpha = degrees(alpha)

    elif Ix != 0 and sqrt(Ix**2 + Iy**2) > 1:
        beta = degrees(atan(Iy/Ix))
        alpha = 35
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) > 1:
        alpha = 35
        if Iy > 0:
            beta = 90
        elif Iy < 0:
            beta = 270

    if alpha > 35:
        alpha = 35

    alpha = prevAlpha * omega + (1-omega) * alpha
    beta = prevBeta * omega + (1-omega) * beta

    alpha = round(round(alpha / 0.2) * 0.2, -int(floor(log10(0.2))))   ## Round to +-0.2
    beta = round(round(beta / 0.2) * 0.2, -int(floor(log10(0.2))))

    if alpha <= 35 and beta <= 360 and arduinoIsConnected == True and startBalanceBall == True:
        ser.write((str(dataDict[(alpha,beta)])+"\n").encode())

    #debugging lines:
    #print(alpha, beta)
    #print(Ix,Iy,alpha,beta,ballPosX,ballPosY,prevBallPosX,prevBallPosY,sommeErreurX,sommeErreurY)

    if startBalanceBall == True:
        sommeErreurX += (targetX-ballPosX)
        sommeErreurY += (targetY-ballPosY)


prevX,prevY = 0,0
prevTargetX, prevTargetY = 0,0
start_time = 0
lastFPSUpdateTime = 0
framesSinceUpdate = 0
def main():
    global H,S,V
    global getPixelColor
    global x,y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevTargetX, prevTargetY
    global targetX, targetY, sommeErreurX, sommeErreurY
    global camWidth,camHeight
    global timeInterval, start_time, lastFPSUpdateTime, framesSinceUpdate
    
    _, img=cam.read()
    img = img[0:int(camHeight),int((camWidth-camHeight)/2):int(camWidth-((camWidth-camHeight)/2))] #[Y1:Y2,X1:X2]
    imgCircle = np.zeros(img.shape, dtype=np.uint8)
    cv2.circle(imgCircle, (240,240), 270, (255, 255, 255), -1, 8, 0)
    img = img & imgCircle
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    if getPixelColor == True and mouseY > 0 and mouseY < 480 and mouseX < 480:
        pixelColorOnClick = img[mouseY,mouseX]
        pixelColorOnClick = np.uint8([[pixelColorOnClick]])
        pixelColorOnClick = cv2.cvtColor(pixelColorOnClick,cv2.COLOR_BGR2HSV)
        H = pixelColorOnClick[0,0,0]
        S = pixelColorOnClick[0,0,1]
        V = pixelColorOnClick[0,0,2]
        print(mouseX, mouseY)
        getPixelColor = False
    
    lowerBound=np.array([H-sliderH.get(),S-sliderS.get(),V-sliderV.get()])
    upperBound=np.array([H+sliderH.get(),S+sliderS.get(),V+sliderV.get()])

    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    mask = cv2.blur(mask,(6,6))
    mask = cv2.erode(mask, None, iterations=2)         # reduce noise
    mask = cv2.dilate(mask, None, iterations=2)        # reduce noise
    
    cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    center = None

    cv2.circle(img, (int(targetX), int(targetY)), int(4),(255, 0, 0), 2)
    if showCalqueCalibrationBool == True:
        cv2.circle(img, (240,240), 220,(255, 0, 0), 2)
        cv2.circle(img, (240,240), 160,(255, 0, 0), 2)
        cv2.line(img, (240, 240), (240, 240+160), (255,0,0), 2)
        cv2.line(img, (240, 240), (240+138, 240-80), (255,0,0), 2)
        cv2.line(img, (240, 240), (240-138, 240-80), (255,0,0), 2)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        timeInterval = time.time() - start_time
        (x, y), radius = cv2.minEnclosingCircle(c)
        if radius > 10:
            cv2.putText(img,str(int(x)) + ";" + str(int(y)).format(0, 0),(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
            cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            PIDcontrol(int(x),int(y),prevX,prevY,targetX,targetY)
            start_time = time.time()
    else:
        sommeErreurX, sommeErreurY = 0,0

    if showVideoWindow:
        videoWindow.deiconify()
        BRetourVideo.configure(text="Close camera preview")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.image = imgtk
        lmain.configure(image=imgtk)
    else:
        videoWindow.withdraw()
        BRetourVideo.configure(text="Open camera preview")

    setTargetForMovement()
    if prevTargetX != targetX or prevTargetY != targetY:
        sommeErreurX, sommeErreurY = 0,0

    paintGraph()
    prevX,prevY = int(x), int(y)
    prevTargetX, prevTargetY = targetX, targetY
    prevAlpha = alpha
    prevBeta = beta

    framesSinceUpdate += 1
    now = time.time()
    if now > lastFPSUpdateTime + 1.0:
        fps = float(framesSinceUpdate) / (now - lastFPSUpdateTime)
        FPSLabel.configure(text="%0.1f FPS" % (fps))
        lastFPSUpdateTime = now
        framesSinceUpdate = 0

    lmain.update()
    lmain.after_idle(main)
    #lmain.after(50, main)



FrameVideoControl = tk.LabelFrame(controllerWindow, text="Camera control")
FrameVideoControl.place(x=20,y=20,width=380)

FrameVideoInfo = tk.Frame(FrameVideoControl)
FPSLabel = tk.Label(FrameVideoInfo, text="FPS", anchor=tk.NW, pady=0)
FPSLabel.pack(side=tk.LEFT, anchor=tk.W, padx=15, expand=True)
BRetourVideo = tk.Button(FrameVideoInfo, text="", command=toggleCameraFrameWindow)
BRetourVideo.pack(side=tk.LEFT)
BPositionCalibration = tk.Button(FrameVideoInfo, text="Calque", command=showCalqueCalibration)
BPositionCalibration.pack(side=tk.LEFT, anchor=tk.E, padx=15)
FrameVideoInfo.pack(fill=tk.X)

sliderH = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensitivity H", length=350, tickinterval = 10)
sliderH.set(sliderHDefault)
sliderH.pack()
sliderS = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensitivity S", length=350, tickinterval = 10)
sliderS.set(sliderSDefault)
sliderS.pack()
sliderV = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensitivity V", length=350, tickinterval = 10)
sliderV.set(sliderVDefault)
sliderV.pack()



FrameServosControl = tk.LabelFrame(controllerWindow, text="Servo control")
FrameServosControl.place(x=20,y=315,width=380)
BLowerPlate = tk.Button(FrameServosControl, text="Lower plate", command=lowerPlate)
BLowerPlate.pack()
BRaisePlate = tk.Button(FrameServosControl, text="Raise plate", command=raisePlate)
BRaisePlate.pack()
BTesterServos = tk.Button(FrameServosControl, text="Test servos", command=servosTest)
BTesterServos.pack()
BStartBalance = tk.Button(FrameServosControl, text="Start", command=startBalance, highlightbackground = "#36db8b")
BStartBalance.pack()



FramePIDCoef = tk.LabelFrame(controllerWindow, text="PID coefficients")
FramePIDCoef.place(x=420,y=20,width=380)
BafficherGraph = tk.Button(FramePIDCoef, text="", command=toggleGraphWindow)
BafficherGraph.pack()
sliderCoefP = tk.Scale(FramePIDCoef, from_=0, to=15, orient="horizontal", label="P", length=350, tickinterval = 3, resolution=0.01)
sliderCoefP.set(sliderCoefPDefault)
sliderCoefP.pack()
sliderCoefI = tk.Scale(FramePIDCoef, from_=0, to=1, orient="horizontal", label="I", length=350, tickinterval = 0.2, resolution=0.001)
sliderCoefI.set(sliderCoefIDefault)
sliderCoefI.pack()
sliderCoefD = tk.Scale(FramePIDCoef, from_=0, to=10, orient="horizontal", label="D", length=350, tickinterval = 2, resolution=0.01)
sliderCoefD.set(sliderCoefDDefault)
sliderCoefD.pack()


FrameBallControl = tk.LabelFrame(controllerWindow, text="Ball control")
FrameBallControl.place(x=420,y=315,width=380, height= 132)
BballHold = tk.Radiobutton(FrameBallControl, text="Hold the ball on target", value=MODE_HOLD_ON_TARGET)
BballHold.pack()
BballHold.select()
BballCircle = tk.Radiobutton(FrameBallControl, text="Move the ball in a circle", value=MODE_CIRCLE)
BballCircle.pack()
BballEight = tk.Radiobutton(FrameBallControl, text="Move the ball in a figure eight", value=MODE_EIGHT)
BballEight.pack()

label = tk.Label(controllerWindow, text="Arduino disconnected  ", fg="red", anchor="ne")
label.pack(fill="both")
BReset = tk.Button(controllerWindow, text = "Reset", command = resetSlider)
BReset.place(x=20, y=460)
BConnect = tk.Button(controllerWindow, text = "Connect", command = connectArduino, background="black")
BConnect.place(x=100, y=460)
BQuit = tk.Button(controllerWindow, text = "Quit", command = endProgam)
BQuit.place(x=730, y=460)


showGraphPositionX = tk.IntVar()
showGraphPositionX.set(1)
CheckbuttonPositionX = tk.Checkbutton(graphWindow, text="X", variable=showGraphPositionX, command=refreshGraph)
CheckbuttonPositionX.place(x=500,y=20)
showGraphPositionY = tk.IntVar()
showGraphPositionY.set(1)
CheckbuttonPositionY = tk.Checkbutton(graphWindow, text="X", variable=showGraphPositionY, command=refreshGraph)
CheckbuttonPositionY.place(x=500,y=40)
showGraphAlpha = tk.IntVar()
CheckbuttonAlpha = tk.Checkbutton(graphWindow, text="Plate angle", variable=showGraphAlpha, command=refreshGraph)
CheckbuttonAlpha.place(x=500,y=60)



videoWindow.protocol("WM_DELETE_WINDOW", toggleCameraFrameWindow)
videoWindow.bind("<Button-2>", getMouseClickPosition)
videoWindow.bind("<Button-1>", setTargetWithMouse)

main()
# Force the main window to the top
# TODO make it focussed
controllerWindow.call('wm', 'attributes', '.', '-topmost', '1')
controllerWindow.call('wm', 'attributes', '.', '-topmost', '0')

tk.mainloop()






