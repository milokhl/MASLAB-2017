from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder, Gyro, Motor, AnalogInput, Color, Servo
import numpy as np
import cv2
import math
from graphics import *

#Wall follow with the wall on the left of the robot until it sees a block.
#Does not yet incorporate a pause and rotate during wall following.
#When it sees a block, it stops wall following and goes to pick it up.

class DriveStraight(SyncedSketch):
    #motor pins
    left_motor_dir_pin, left_motor_pwm_pin = 4,5#2,3
    right_motor_dir_pin, right_motor_pwm_pin = 2,3#4,5
    #encoder pins
    left_motor_encoder_pins = 16,15#6, 7
    right_motor_encoder_pins = 6,7#8, 9
    #gyroscope pin
    ss_pin = 10
#    #IR sensor pins
#    adc_pins = [0,1,2,3] #[front,left,right,back]
    
    #set the initial motor speeds for drive wheels, block positioner, and elevator
    init_motor_speed = 50
    init_elevator_speed = 255
    init_bp_speed = 255

    
    #Initial values for calculating estimated theta
    desired_theta = 0.0
    theta_gyro_old = 0.0
    theta_est = 0.0
    #physical constants for using encoder to estimate theta
    left_motor_encoder_old = 0.0
    right_motor_encoder_old = 0.0
    axel_length = 11 #distance between two wheels
    wheel_radius = 1.435 #radius of wheels
    wheel_circumference = 2*wheel_radius*np.pi
    counts_per_rotation = 3200.0 #from the motor ecoder specs
    #using encoder to calculate distance travelled when block collecting
    left_motor_encoder_old2 = 0.0
    right_motor_encoder_old2 = 0.0
    left_desired_encoder = 0.0
    right_desired_encoder = 0.0
    bias = 0.0
    power = 0.0
    extra = 2000 #extra encoder rotation so block actually gets in
    #Values for PID calculations
    last_diff = 0.0
    integral = 0.0
    derivative = 0.0
    dt = 0.0
    alpha = 0.9
    kP = 2
    kI = 1e-3
    kD = 110
    largeAngle = 20
    maxTurnSpeed = 50

    #How often to perform calculations in milliseconds
    refresh_rate = 10

    #Parameters for webcam
    webcamNumber = -1
    webcamWidth = 160
    webcamHeight = 120
    redRatio = 1.9
    greenRatio = 1.15
    blueRatio = 1.32
    purpleRatio = 1.1
    blockCenteringTolerance = 5
    CAMERA_CENTER = (webcamWidth/2,webcamHeight/2)
    #For when rotating without block in site
    deltaTheta = 2
    #When a block is seen rotate slower
    deltaThetaBlock = 0.5

    #How long to drive when block leaves frame
    timeToDrive = 2000
    shouldDrive = False
    countdown = 0
    
    #Variables For Block Collection
    step = 0
    shouldCheck = False
    backUpDist = 6.0
    hasChecked = False
    shouldSkipCheck = False

    #Variables For Wall Following
    previousSensorVals = [0.0,0.0,0.0,0.0]
    wallFollowDist = 7.0
    wallFollowTolerance = 2.0
    
    #State Variables
    blockSighted = False
    centerPlatformSighted = False
    onWall = False
    scanStartAngle = 0.0
    shouldScan = False
    numberOfOurs = 0
    numberOfOpponents = 0
    #The color of our blocks. Red or Green. Maybe replace with a switch?
    ourColor = "Red"
    
    SORT_SERVO_PIN = 29
    opponentAngle = 85
    ourAngle = 38
    sortRedRatio = 1.6
    sortGreenRatio = 1.1
    
    state = 0    
    scanState = 0
    sortingState = 0
    pickUpState = 0
    
    #Calculate location
    initialPos1 = (0,0,0) #x,y,theta
    left_encoder_prev1 = 0
    right_encoder_prev1 = 0
    
    initialPos2 = (0,0,0) #x,y,theta
    left_encoder_prev2 = 0
    right_encoder_prev2 = 0
    
    initialPos3 = (0,0,0) #x,y,theta
    left_encoder_prev3 = 0
    right_encoder_prev3 = 0
    
    #Target
    targets = [(0,0,0),(12,0,90),(12,12,180),(0,12,270)]
    target = targets[0]
    driveToTargetState = 0
    straightCount = 0
    part1=True
    
    
    def setup(self):
        assert self.ourColor.lower()=="red" or self.ourColor.lower=="green"
        
        #set up timer
        self.timer = Timer()
        self.timer2 = Timer()
        self.timer3 = Timer()
        self.statusTimer = Timer()
        
        #set up motors
        self.left_motor = Motor(self.tamp, self.left_motor_dir_pin, self.left_motor_pwm_pin)
        self.right_motor = Motor(self.tamp, self.right_motor_dir_pin, self.right_motor_pwm_pin)
        self.motorval_left = 0
        self.motorval_right = 0
        print "drive motor objects made"
        
        #set up encoders
        self.left_motor_encoder = Encoder(self.tamp, *self.left_motor_encoder_pins, continuous=True)
        self.right_motor_encoder = Encoder(self.tamp, *self.right_motor_encoder_pins, continuous=True)
        print "encoder objects made"

        #set up the gyro
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        print "gyro object made"

        #set up the webcam
#        self.cap = cv2.VideoCapture(self.webcamNumber)
#        self.cap.set(3,self.webcamWidth)
#        self.cap.set(4,self.webcamHeight)

#        #set up the IR sensors
#        self.frontSensor = AnalogInput(self.tamp, self.adc_pins[0])
#        self.leftSensor = AnalogInput(self.tamp, self.adc_pins[1])
#        self.rightSensor = AnalogInput(self.tamp, self.adc_pins[2])
#        self.backSensor = AnalogInput(self.tamp, self.adc_pins[3])

        #start all motors
        self.left_motor.write(self.motorval_left<0, abs(self.motorval_left))
        self.right_motor.write(self.motorval_right<0, abs(self.motorval_right))
        
        #set up the visualization
        self.win = GraphWin('RobotTracking', 500, 500)
        self.win.setCoords(0, 0, 500, 500)
        start = (10,10)
        delta = 5.0
        corners = [Point(start[0]+delta,start[1]),Point(start[0]-delta,start[1]-delta/2),Point(start[0]-delta,start[1]+delta/2)]
        self.robot = Polygon(*corners)
        self.robot.setFill('red')
        self.robot.draw(self.win)
        self.robotCenter = Point(start[0],start[1])
        self.robotCenter.setFill('white')
        self.robotCenter.draw(self.win)

    def loop(self):
        if self.timer3.millis()<175*1000:
            if self.timer.millis() > self.refresh_rate:
                self.dt = self.timer.millis()
                self.timer.reset()
#                _,frame = self.cap.read()
#                self.updateThetaEstNoEncoder()
#                self.calculatePower()
#                if self.last_diff<.5:
#                    self.bias=25
#                else:
#                    self.bias=0
#                if self.initialPos1[0]>=12:
#                    self.desired_theta = -90.
#                if self.initialPos1[1]>=12:
#                    self.bias = 0
#                self.updateMotors()
                if self.part1==True:
                    self.target=(24,48,90)
#                    self.target=(0,0,90)
                if self.state == 0:
                    result = self.driveToTarget()
                    if result:
                        self.part1=False
                        if self.target==(0,12,0):                            
                            self.state=-1
#                            self.win.close()
                        else:
                            self.target=(0,12,0)
#                        try:
#                            self.target=self.targets[self.targets.index(self.target)+1]
#                        except:
#                            self.target = self.targets[0]
#                elif self.state == 1:
#                    self.pickUpBlock()
#            if self.statusTimer.millis()>=500:
#                self.statusTimer.reset()
                #Print status
                print "overall state:",self.state
                print "driveToTarget state:",self.driveToTargetState,"straightCount:",self.straightCount
                print "target:",self.target
                #Update and print positions
                self.updateAllPos()
                print "theta_est:",self.theta_est,"desired_theta:",self.desired_theta
                print "visual position:",self.robotCenter.getX(),self.robotCenter.getY()
                #print angle from encoders
                self.getAngles()
                #print "power:",self.power, ", gyro:",self.gyro.val, ", theta_est:", self.theta_est
                #print "last_diff",self.last_diff,"dt:",self.dt, "desired_theta", self.desired_theta
                #print "integration:", self.integral, "derivative:",self.derivative
                #print other stuff
                print "left motor:",self.motorval_left,"right motor",self.motorval_right
                print "bias:",self.bias
                print "left encoder:",self.left_motor_encoder.val,"right encoder:", self.right_motor_encoder.val
                print "left desired:", self.left_desired_encoder,"right desired:", self.right_desired_encoder
                
                print "------------------.---------"
	else:
		#start all motors
		self.left_motor.write(self.motorval_left<0, 0)
		self.right_motor.write(self.motorval_right<0, 0)

    def driveToTarget(self):
        self.updateThetaEstNoEncoder()
        #get direction and set bias to 0 until we are at the correct angle
        currState = self.initialPos2
        P1 = (currState[0],currState[1])
        P2 = (self.target[0],self.target[1])

        if self.driveToTargetState==0:
            #turn toward the point and drive
            angle = -math.degrees(self.getDesiredAngle(P1,P2))
#            print "angle:",angle
#            print "deltaTheta:",abs(self.theta_est - angle)
            #don't drive unless we are pointing in the correct direction
            if abs(self.theta_est - angle) <= 10:
                self.driveToTargetState=1
                
            if self.theta_est<angle:
                self.desired_theta = self.theta_est + 5
            elif self.theta_est>angle:
                self.desired_theta = self.theta_est - 5
                    
#            self.desired_theta = -math.degrees(angle)
            self.bias = 0

        elif self.driveToTargetState==1:
            angle = -math.degrees(self.getDesiredAngle(P1,P2))
#            print "angle:",angle
#            print "deltaTheta:",abs(self.theta_est - angle)
            self.desired_theta = angle
            self.bias = self.init_motor_speed
            #switch states if we are within 1 inch of our goal
            if abs(self.theta_est - angle) > 10:
                self.driveToTargetState=0
                
            if math.hypot(P2[0]-P1[0], P2[1]-P1[1])<=1:
                self.driveToTargetState = 2
                self.bias = 0
        elif self.driveToTargetState==2:
            if type(self.target[2])==float or type(self.target[2])==int:
                angle = -self.target[2]
            else:
                angle = self.theta_est
#            print "angle:",angle
#            print "deltaTheta:",abs(self.theta_est - angle)
            #point in the direction specified in the target
#            self.desired_theta = -self.target[2]
            self.bias = 0
            if abs(self.theta_est - angle) < 10:
                self.desired_theta = angle
                if abs(self.desired_theta-self.theta_est)<3:
                    self.straightCount += 1
                elif abs(self.desired_theta-self.theta_est)>=3:
                    self.straightCount = 0
                if self.straightCount == 10:
                    self.desired_theta = self.theta_est
#                    self.calculatePower()
                    self.power = 0
                    self.updateMotors()
                    self.driveToTargetState = 0
                    self.straightCount = 0
                    return True
            else:
#                if abs(self.theta_est - self.desired_theta) <= 60:
#                    if self.desired_theta<-self.target[2]:
#                        self.desired_theta += 5
#                    elif self.desired_theta>-self.target[2]:
#                        self.desired_theta -= 5
                if self.theta_est<angle:
                    self.desired_theta = self.theta_est + 10
                elif self.theta_est>angle:
                    self.desired_theta = self.theta_est - 10        
        self.calculatePower()
        self.updateMotors()
        return False

    def getDesiredAngle(self,P1,P2):
        x,y = P2[0]-P1[0],P2[1]-P1[1]
#        print "x,y:",x,y
        if y==0 and x==0:
            angle = 0.
        elif y==0:
            if x>0:
                angle = 0.
            else:
                angle = math.pi
        elif x==0:
            if y>0:
                angle = math.pi/2
            else:
                angle = 3*math.pi/2
        else:
            angle = math.atan(y/x)
            if x<0:
                angle += math.pi
#        print "desired angle:",math.degrees(angle)
        return angle

    def updateAllPos(self):
        self.updateInitPos1()
        self.updateInitPos2()
        self.updateInitPos3()
        print "version 1:",self.initialPos1
        print "version 2:",self.initialPos2
        print "version 3:",self.initialPos3
        try:
            print "estAxel:",(self.axel_length*self.initialPos3[2]/self.initialPos1[2])
        except:
            pass
        
    def updateInitPos1(self):
        #assume we went in a straight line at every time step in the direction of theta
        
        deltaLeft = self.wheel_circumference*((self.left_motor_encoder.val - self.left_encoder_prev1)/self.counts_per_rotation) #number of rotations*circumference
        deltaRight = self.wheel_circumference*((self.right_motor_encoder.val - self.right_encoder_prev1)/self.counts_per_rotation) #number of rotations*circumference
        dist = (deltaLeft+deltaRight)/2.0
        angle = math.radians(self.initialPos1[2])
        self.updateThetaEstNoEncoder()
        self.initialPos1 = (self.initialPos1[0] + dist*math.cos(angle),self.initialPos1[1] + dist*math.sin(angle),-self.theta_est)
        
        self.left_encoder_prev1 = self.left_motor_encoder.val
        self.right_encoder_prev1 = self.right_motor_encoder.val
        
    def rotateRobot(self,deltaTheta):
        #rotates a triangle a certain deltaTheta radians
        #???
        points = self.robot.getPoints()
        newPoints = []
        for point in points:
            x2 = self.robotCenter.getX() + (point.getX() - self.robotCenter.getX())*math.cos(deltaTheta) - (point.getY() - self.robotCenter.getY())*math.sin(deltaTheta)
            y2 = self.robotCenter.getY() + (point.getY() - self.robotCenter.getY())*math.cos(deltaTheta) + (point.getX() - self.robotCenter.getX())*math.sin(deltaTheta)
            newPoints.append(Point(x2,y2))
        self.robot.undraw()
        self.robotCenter.undraw()
        robot = Polygon(*newPoints)
        robot.setFill('red')
        robot.draw(self.win)
        self.robotCenter.draw(self.win)
        self.robot = robot

    def updateInitPos2(self):
        #using equations for differential steering, but replacing the angle with the value from the gyroscope
        #http://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
        self.updateThetaEstNoEncoder()
        x = self.initialPos2[0]
        y = self.initialPos2[1]
        heading = math.radians(self.initialPos2[2])
        
        leftEnc = self.left_motor_encoder.val
        rightEnc = self.right_motor_encoder.val
        
        leftDelta = float(self.wheel_circumference)*((leftEnc - self.left_encoder_prev2)/self.counts_per_rotation) #number of rotations*circumference
        rightDelta = float(self.wheel_circumference)*((rightEnc - self.right_encoder_prev2)/self.counts_per_rotation) #number of rotations*circumference
        if abs(leftDelta-rightDelta)<=1e-6:
            new_x = x + leftDelta*math.cos(heading)
            new_y = y + rightDelta*math.sin(heading)
            
        else:
            R = self.axel_length*(leftDelta+rightDelta)/(2.0*(rightDelta-leftDelta))
            wd = (rightDelta-leftDelta)/self.axel_length
            
            new_x = x + R * math.sin(wd + heading) - R * math.sin(heading)
            new_y = y - R * math.cos(wd + heading) + R * math.cos(heading)

        new_heading = -self.theta_est
        self.initialPos2 = (new_x,new_y,new_heading)

        dx = (self.initialPos2[0] - self.robotCenter.getX())
        dy = (self.initialPos2[1] - self.robotCenter.getY())
        startPoint  = self.robotCenter.clone()
        self.robot.move(dx,dy)
        self.robotCenter.move(dx,dy)
        deltaTheta = math.radians(self.initialPos2[2]) - heading
        self.rotateRobot(deltaTheta)
        endPoint  = self.robotCenter.clone()
        trail = Line(startPoint,endPoint) # set endpoints
        trail.setWidth(1)
        trail.setFill('blue')
        trail.draw(self.win)           
        
        self.left_encoder_prev2 = leftEnc
        self.right_encoder_prev2 = rightEnc
        
    def updateInitPos3(self):
        #purely encoders using equations for differential steering
        #http://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
        self.updateThetaEstNoEncoder()
        x = self.initialPos3[0]
        y = self.initialPos3[1]
        heading = math.radians(self.initialPos3[2])
        
        leftEnc = self.left_motor_encoder.val
        rightEnc = self.right_motor_encoder.val
        
        leftDelta = float(self.wheel_circumference)*((leftEnc - self.left_encoder_prev3)/self.counts_per_rotation) #number of rotations*circumference
        rightDelta = float(self.wheel_circumference)*((rightEnc - self.right_encoder_prev3)/self.counts_per_rotation) #number of rotations*circumference
        if abs(leftDelta-rightDelta)<=1e-6:
            new_x = x + leftDelta*math.cos(heading)
            new_y = y + rightDelta*math.sin(heading)
            new_heading = heading
        else:
            R = self.axel_length*(leftDelta+rightDelta)/(2.0*(rightDelta-leftDelta))
            wd = math.radians(-self.theta_est) - heading
            print "dx:", R * math.sin(wd + heading) - R * math.sin(heading)
            print "dy:", -R * math.cos(wd + heading) + R * math.cos(heading)
            new_x = x + R * math.sin(wd + heading) - R * math.sin(heading)
            new_y = y - R * math.cos(wd + heading) + R * math.cos(heading)
            new_heading = heading + wd
#        while new_heading<0 or new_heading>2*math.pi:
#            if new_heading<0:
#                new_heading = new_heading + 2*math.pi
#            elif new_heading>2*math.pi:
#                new_heading = new_heading - 2*math.pi
        self.initialPos3 = (new_x,new_y,math.degrees(new_heading))
        self.left_encoder_prev3 = leftEnc
        self.right_encoder_prev3 = rightEnc
    
    def getAngles(self):
        left_arc = self.wheel_circumference*(self.left_motor_encoder.val/self.counts_per_rotation) #number of rotations*circumference
        right_arc = self.wheel_circumference*(self.right_motor_encoder.val/self.counts_per_rotation) #number of rotations*circumference
        angleEnc = math.degrees(((left_arc - right_arc)/self.axel_length)) #converted to degrees
        print "angle from encoders:",angleEnc
        print "angle from gyro:",self.gyro.val
        
    def updateThetaEstWithEncoder(self):
        #get gyro angle change
        theta_gyro_new = self.gyro.val
        change_gyro = theta_gyro_new - self.theta_gyro_old
        self.theta_gyro_old = theta_gyro_new
        #get encoder angle
        self.left_motor_encoder_new = self.left_motor_encoder.val
        self.right_motor_encoder_new = self.right_motor_encoder.val
        left_arc = self.wheel_circumference*((self.left_motor_encoder_new - self.left_motor_encoder_old)/self.counts_per_rotation) #number of rotations*circumference
        right_arc = self.wheel_circumference*((self.right_motor_encoder_new - self.right_motor_encoder_old)/self.counts_per_rotation) #number of rotations*circumference
        change_enc = math.degrees(((left_arc - right_arc)/self.axel_length)) #converted to degrees
        self.left_motor_encoder_old = self.left_motor_encoder_new
        self.right_motor_encoder_old = self.right_motor_encoder_new
        #calculate the new angle
        self.theta_est = self.theta_est + change_gyro*self.alpha + change_enc*(1-self.alpha)
        
    def updateThetaEstNoEncoder(self):
        #Only use gyroscope to estimate current angle
        self.theta_est = self.gyro.val

    def calculatePower(self):
        #Use PID and update motors
        diff = self.desired_theta - self.theta_est
        self.integral = self.integral + diff*self.dt
        self.derivative = (diff - self.last_diff)/self.dt
        self.power = self.kP*diff + self.kI*self.integral + self.kD*self.derivative
        self.last_diff = diff

    def updateMotors(self):
        #Prevent a power that will try to set the wheels above or below possible values
        if ((self.bias + self.power)>255) or ((self.bias - self.power)<-255):
            self.power = 255 - abs(self.bias)
        elif ((self.bias + self.power)<-255) or ((self.bias - self.power)>255):
            self.power = -255 + abs(self.bias)
            
        #Stop power from being too high so the change in wheel speed is not too drastic
        if self.power>self.maxTurnSpeed:
            self.power = self.maxTurnSpeed
        elif self.power<-self.maxTurnSpeed:
            self.power = -self.maxTurnSpeed
            
        #Set motor values
        self.motorval_left = self.bias + self.power
        self.motorval_right = self.bias - self.power
        self.left_motor.write(self.motorval_left<0, abs(self.motorval_left))
        self.right_motor.write(self.motorval_right<0, abs(self.motorval_right))
        


    #Vision and Block Collection
    def generate_black_image(self):
        #Make a blank black screen to add colored pixels to
        image = np.zeros((self.webcamHeight,self.webcamWidth,3),np.uint8)
        image[:] = (0,0,0)
        return image
    def redgreen_scan(self,frame):
        newFrame = self.generate_black_image() #start off the new frame as all black
        red_pixels = []
        green_pixels = []

        for xPos in xrange(0,self.webcamWidth): #Loop through rows
            for yPos in xrange(0,self.webcamHeight): #Loop through column
                    if frame.item(yPos,xPos,2)>self.redRatio*frame.item(yPos,xPos,0)\
                       and frame.item(yPos,xPos,2)>self.redRatio*frame.item(yPos,xPos,1):
                        newFrame.itemset((yPos,xPos,2),255)
                        red_pixels.append((xPos,yPos)) #add the red pixel to the red pixel list

                    elif frame.item(yPos,xPos,1)>self.greenRatio*frame.item(yPos,xPos,0)\
                         and frame.item(yPos,xPos,1)>self.greenRatio*frame.item(yPos,xPos,2):
                        newFrame.itemset((yPos,xPos,1), 255)
                        green_pixels.append((xPos,yPos)) #add green pixel to the list
            
        return (newFrame, red_pixels, green_pixels)
    def blue_scan(self,frame):
        newFrame = self.generate_black_image()
        blue_pixels = []
        for xPos in range(0,self.webcamWidth): #Loop through rows
            for yPos in range(0,self.webcamHeight): #Loop through columns
                if (frame.item(yPos,xPos,0)>self.blueRatio*frame.item(yPos,xPos,2)\
                    and frame.item(yPos,xPos,0)>self.blueRatio*frame.item(yPos,xPos,1)\
                    and not(frame.item(yPos,xPos,2)>self.purpleRatio*frame.item(yPos,xPos,1))):
                    newFrame.itemset((yPos,xPos,0),255)
                    blue_pixels.append((xPos,yPos)) #add the blue pixel to the blue pixel list
        return (newFrame, blue_pixels)
    def purple_scan(self,frame):
        newFrame = self.generate_black_image()
        purple_pixels = []
        for xPos in range(0,self.webcamWidth): #Loop through rows
            for yPos in range(0,self.webcamHeight): #Loop through columns
                if (frame.item(yPos,xPos,0)>self.purpleRatio*frame.item(yPos,xPos,1)\
                    and frame.item(yPos,xPos,2)>self.purpleRatio*frame.item(yPos,xPos,1)\
                    and frame.item(yPos,xPos,2)<75):
                    newFrame.itemset((yPos,xPos,0),255)
                    newFrame.itemset((yPos,xPos,2),255)
                    purple_pixels.append((xPos,yPos))
        return (newFrame, purple_pixels)

    def getContours(self, frame):
        #Takes in an image and finds the contours. Returns the largest contour of
        #a certain size (ideally a block)
        image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(image,0,255,0)
        w,allContours,hierarchy = cv2.findContours(thresh, 1, 2)
        contours = sorted(allContours,key=cv2.contourArea,reverse=True)[:1]
        for cnt in contours:
            if 100.0*cv2.contourArea(cnt)/(self.webcamWidth*self.webcamHeight) < 0.3:
                contours.remove(cnt)
        if len(contours)==0:
            return None
        return contours[0]

    def determine_distance(self,pixel_width):
            """
            See: http://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
            """
            FOCAL_LENGTH = 376 #calculated experimentally
            OBJ_WIDTH = 2 #2in blocks

            try:
                    Dist = (float(OBJ_WIDTH)*FOCAL_LENGTH)/pixel_width
                    return Dist
            except:
                    return 0.0

    def determine_angle(self, block_height_pixels, pixels_from_ctr, block_dist):
            try:
                    dist_from_ctr = (float(pixels_from_ctr) / block_height_pixels) * 2
                    angle = math.asin(dist_from_ctr/block_dist)
                    angle_deg = math.degrees(angle)
                    return angle_deg
            except:
                    return 0.0

    def getDistAngle(self, contour):
        if contour==None:
            return None, None
        #Get center of the block
        x,y,w,h = cv2.boundingRect(contour)
        #rect = cv2.rectangle(redgreen_image, (x,y), (x+w, y+h), (255,0,0), 2)
        cx = x + w/2
        cy = y + h/2
        CENTER = (cx, cy)
        #DELTA_X IS THE DISTANCE (IN PIXELS) OF THE OBJECT AWAY FROM THE CENTER OF VISION
        DELTA_X = self.CAMERA_CENTER[0] - CENTER[0]
        dist = self.determine_distance(h)
        angle = self.determine_angle(h, DELTA_X, dist)
        return dist, angle

    def checkForBlock(self):
        print "Searching for block."
        _,frame = self.cap.read()
        redgeenFrame = self.redgreen_scan(frame)[0]
        contour = self.getContours(redgeenFrame)
        #if there is no block
        if contour==None:
            print "No block found"
            self.blockSighted = False
            self.shouldSkipCheck = False
            return False
        #if there is a block
        else:
            self.step = 0
            self.blockSighted = True
            self.shouldSkipCheck = True
            self.shouldScan = False
            print "Block found"
            return True
            
    def encodersAboveDesired(self):
        #Returns true if the sum of the encoder values are greater than or equal to the sum of the desired values.
        #True if you are in front of or at where you wanted to travel.
        return self.left_motor_encoder.val+self.right_motor_encoder.val>=self.left_desired_encoder+self.right_desired_encoder
    
    
    def pickUpBlock(self):
        self.updateThetaEstNoEncoder()
        
        #First check for a block
        if self.pickUpState == 0:
            _,frame = self.cap.read()
            redgeenFrame = self.redgreen_scan(frame)[0]
            contour = self.getContours(redgeenFrame)
            dist, angle = self.getDistAngle(contour)
            #if there is no block
            if dist==None:
                print "No block detected!"
                self.pickUpState = 0
                self.state = 0
                return 
            #If there is a block, set the desired encoders to how far we want to roll.
            #We look at the sum of the encoders to see how far we actually moved because
            #spinning in place increases one while decreasing the other and that screws up the math
            print "dist:",dist,"angle:",angle
            self.desired_theta = self.theta_est - angle
            self.left_motor_encoder_old2 = self.left_motor_encoder.val
            self.right_motor_encoder_old2 = self.right_motor_encoder.val
            self.left_desired_encoder = dist*3200.0/self.wheel_circumference + self.left_motor_encoder_old2 + self.extra
            self.right_desired_encoder = dist*3200.0/self.wheel_circumference + self.right_motor_encoder_old2 + self.extra
            self.pickUpState = 1
        elif self.pickUpState ==1:
            self.calculatePower()
            #If we are not centered on a block, don't move.
            if abs(self.last_diff)>=self.blockCenteringTolerance:
                self.bias = 0.0            
            #Move forward to collect the block
            else:
                self.bias = self.init_motor_speed
                if self.encodersAboveDesired():
                    self.pickUpState = 2
        elif self.pickUpState ==2:
            #You reached the block. Now change the desired encoders to go in reverse. We need to back up to check that we actually got the block.
            self.left_motor_encoder_old2 = self.left_motor_encoder.val
            self.right_motor_encoder_old2 = self.right_motor_encoder.val
            self.left_desired_encoder = -self.backUpDist*3200.0/self.wheel_circumference + self.left_motor_encoder_old2
            self.right_desired_encoder = -self.backUpDist*3200.0/self.wheel_circumference + self.right_motor_encoder_old2
            self.bias = 0.0
            self.pickUpState = 3
        elif self.pickUpState==3:
            self.calculatePower()
            #Reverse until we reached our desired. We need to get any potential missed blocks in our field of view.
            if abs(self.last_diff)>=self.blockCenteringTolerance:
                self.bias = 0.0            
            #Move forward to collect the block
            else:
                self.bias = -self.init_motor_speed
                if not(self.encodersAboveDesired()):
                    self.pickUpState = 0
                    self.state = 0

            #We backed up enough. Now repeat by starting with checking for blocks.

            #Communicate information to the motors
        self.updateMotors()

    #Wall Finding and Following

    def getIRDist(self,IRsensor):
        #Calculated previously
        return 157107/(IRsensor.val-4907.2)
    def getAllSensorDist(self):
        return [self.getIRDist(self.frontSensor),self.getIRDist(self.leftSensor),self.getIRDist(self.rightSensor),self.getIRDist(self.backSensor)]
    def getDistTravelled(self,oldLeftEnc,oldRightEnc,newLeftEnc,newRightEnc):
        return self.wheel_circumference*((newLeftEnc+newRightEnc)-(oldLeftEnc+oldRightEnc))/3200.0
    def checkOnWall(self):
        currentSensorVals = self.getAllSensorDist()
        return currentSensorVals[1]<=(self.wallFollowDist + self.wallFollowTolerance)
    
    def wallFollow(self):
        #get current values
        current_left_encoder = self.left_motor_encoder.val
        current_right_encoder = self.right_motor_encoder.val
        currentSensorVals = self.getAllSensorDist()
        self.updateThetaEstNoEncoder()
        if currentSensorVals[0]<self.wallFollowDist + self.wallFollowTolerance:
            self.calculatePower()
            if abs(self.last_diff)<10:
                self.desired_theta = self.theta_est + 10.0
            self.bias = self.init_motor_speed/2
        else:
            #get the opposite and hypotenuse of the right triangle
            changeInDist = currentSensorVals[1] - self.previousSensorVals[1]
            distTravelled = self.getDistTravelled(self.left_motor_encoder_old2,self.right_motor_encoder_old2,current_left_encoder,current_right_encoder)
            #calculate the angle deviated from straight along the wall. Correct for it.
            angle_rad = math.asin(float(changeInDist)/distTravelled)
            angle_deg = math.degrees(angle_rad)
            self.desired_theta = self.theta_est - angle_deg
            self.calculatePower()
            #Don't move unless we are straight.
            if abs(self.last_diff)>10:
                self.bias = self.init_motor_speed/2
            else:
                self.bias = self.init_motor_speed
        #Send to motors
        self.updateMotors()
        #Update Values
        self.previousSensorVals = currentSensorVals
        self.left_motor_encoder_old2 = current_left_encoder
        self.right_motor_encoder_old2 = current_right_encoder
        
    def findWall(self):
        currentSensorVals = self.getAllSensorDist()
        self.updateThetaEstNoEncoder()
        #Move Forward until we are within 7 inches of a wall
        if currentSensorVals[1]<=(self.wallFollowDist + self.wallFollowTolerance):
            self.onWall = True
        elif currentSensorVals[0]>self.wallFollowDist:
            self.desired_theta = self.theta_est
            self.bias = self.init_motor_speed
            self.calculatePower()
            self.updateMotors()
            self.onWall = False
        elif currentSensorVals[0] <= self.wallFollowDist:
        #Maybe add "self.onWall=True" here
            self.calculatePower()
            if abs(self.last_diff)<10:
                self.desired_theta = self.theta_est + 10
            self.bias = 0
            self.updateMotors()

    #Scanning for blocks code

    def scan360(self):
        self.updateThetaEstNoEncoder()
        if self.scanState == 0:
            self.scanStartAngle = self.theta_est
            self.scanState = 1
        if self.scanState ==1:
            self.calculatePower()
            
            blockExists = self.checkForBlock()
            if blockExists:
                self.bias = 0.0
                self.desired_theta = self.theta_est
                self.scanState = 0
                self.updateMotors()
                self.state = 1
                return True
            if abs(self.theta_est-self.scanStartAngle+360.0)<=10:
                self.desired_theta = self.scanStartAngle+360.0
                self.scanState = 0
                self.state = 0
                return None
            elif abs(self.last_diff) < 90:
                self.desired_theta = self.theta_est + 5
                self.shouldScan = True
                
            self.bias = 0.0
            self.updateMotors()
        return False
        
    def updateSortingState(self):
        result = self.checkBlockColor()
        if result != 0:
            if result == self.ourColor.lower():
                self.sortingState=1
            else:
                self.sortingState=2
    def checkBlockColor(self):
    #    print "rgb:",settings.colorSensor.r,settings.colorSensor.g,settings.colorSensor.b
        if (self.colorSensor.r > self.sortRedRatio*self.colorSensor.g \
            and self.colorSensor.r > self.sortRedRatio*self.colorSensor.b\
            and self.colorSensor.r>=100):
    #        print "Red Block"
            return "red"
        elif (self.colorSensor.g > self.sortGreenRatio*self.colorSensor.r\
              and self.colorSensor.g > self.sortGreenRatio*self.colorSensor.b):
    #        print "Green Block"
            return "green"
        else:
    #        print "No Block"
            return 0
if __name__ == "__main__":
    sketch = DriveStraight(1, -0.00001, 100)
    sketch.run()
