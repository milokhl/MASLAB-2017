# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 22:13:12 2017

@author: jasmine, arinze
"""

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder,Gyro,Motor,Servo, Color
import numpy as np, cv2, Settings as S
import math,winsound,time,random,threading




class BlockSearchV1(SyncedSketch):
    #motor pins
    left_motor_dir_pin, left_motor_pwm_pin = S.left_motor_dir_pin, S.left_motor_pwm_pin
    right_motor_dir_pin, right_motor_pwm_pin = S.right_motor_dir_pin, S.right_motor_pwm_pin
    #encoder pins
    left_motor_encoder_pins = S.left_motor_encoder_pins
    right_motor_encoder_pins = S.right_motor_encoder_pins
    #gyroscope pin
    ss_pin = S.ss_pin
    
    #Servo pins and angles
    GRABBER_PIN = S.GRABBER_PIN
    grabber_angles = S.grabber_angles #in and out
    ELE_PIN = S.ELE_PIN # TBD
    ele_angles = S.ele_angles # up and down
    SKEW_PIN = S.SKEW_PIN
    skew_angles = S.skew_angles #in and out

    DOOR_PIN = S.DOOR_PIN
    door_angles = S.door_angles  #closed and open

    
    #set the initial motor speed
    init_motor_speed = S.init_motor_speed
    #angle we want to turn to
    desired_theta = S.desired_theta
    #physical constants
    shouldGoAgain = True
    theta_est = S.theta_est
    heading = np.radians(theta_est)
    theta_est_enc = S.theta_est_enc

    axel_length = S.axel_length #8.25 distance between two wheels in inches
    wheel_radius = S.wheel_radius #1.9375 radius of wheels in inches
    wheel_circumference = S.wheel_circumference #2*wheel_radius*numpy.pi
    counts_per_rotation = S.counts_per_rotation #3200.0 #from the motor ecoder specs
    refresh_rate = S.refresh_rate #50


    kP = 4#S.kP #2#0.6*kU
    kI = 1#S.kI #1e-3#tU/2
    kD = 0#S.kD #110#tU/8.0
    maxCorrection = S.maxCorrection #50
    last_diff = 0.0
    integral = 0.0

    distTravelled = 0 #inches
    desired_dist = 0 #inches
    oldEncLeft = 0
    oldEncRight = 0
    x = 0.0
    y = 0.0
    
    angle = None
    
    
    dTheta = 10

    rg = S.rg
    rb = S.rb
    gr = S.gr
    gb = S.gb

    colorSenserg = S.colorSenserg # red to green ratio
    colorSenserb = S.colorSenserb
    colorSensegr = S.colorSensegr
    colorSensegb = S.colorSensegb
    ourColor = S.ourColor
    
    minFraction = 0.005 #percent of pixels needed to be considered a block
    endGame = False

    #overallState: 0=spin until sees block, 1=drive to block, 2=collect block  3 = blocks dropped, drive backwards
    overallState = 0
    #drivingToBlockState: 0=find the angle and rotate to the angle of the block, 1 = get the distance, 2 = drive to the block
    drivingToBlockState = 0

    #driving back states: 0 set desired distance and angle, 1 slowly drive back, stay centered, stop driving
    driveBackState= 0

    blockPUS = -2#S.blockPUS #-1 is no block, 0 is our block, 1 is opponent block
    ourBlockPUS = S.ourBlockPUS
    oppBlockPUS = S.oppBlockPUS
    dropState = S.dropState
    
    servoDelayOurs = S.servoDelayOurs #millisec
    servoDelayOpp = S.servoDelayOpp #millisec
    servoDelayDropStack = 1000 #millisec
    
    numOppBlocks = 0
    numOurBlocks = 0
    
    def setup(self):
        print "Starting Setup"
        #set up timer
        self.timer = Timer()
        self.statusTimer = Timer()
        self.roundTimer = Timer()
        self.collectTimer = Timer()
        self.colorTimer = Timer()
        print "timers made"
        
        #set up motors
        self.left_motor = Motor(self.tamp, self.left_motor_dir_pin, self.left_motor_pwm_pin)
        self.right_motor = Motor(self.tamp, self.right_motor_dir_pin, self.right_motor_pwm_pin)
        print "motor objects made"

        self.motorval_left = self.init_motor_speed
        self.motorval_right = self.init_motor_speed
        #set up encoders
        self.left_motor_encoder = Encoder(self.tamp, *self.left_motor_encoder_pins, continuous=True)
        self.right_motor_encoder = Encoder(self.tamp, *self.right_motor_encoder_pins, continuous=True)
        print "encoder objects made"
        
        #set up the gyro
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        self.cali_timer = Timer()
        self.calibration = 0.0
        self.calibrated = True

        print "gyro object made"
        self.theta_gyro_old = self.gyro.val

        #Set up servos
        self.grabber = Servo(self.tamp, self.GRABBER_PIN)
        self.grabber.write(self.grabber_angles[0]) #grabber starts in

        self.elevator = Servo(self.tamp, self.ELE_PIN)
        self.elevator.write(self.ele_angles[0]) #elevator starts up
        
        self.skewer = Servo(self.tamp, self.SKEW_PIN)
        self.skewer.write(self.skew_angles[1]) #skewer starts out
        self.angleToWrite = self.skew_angles[0]
        
        self.door = Servo(self.tamp, self.DOOR_PIN)
        self.door.write(self.door_angles[0]) #door starts closed
        print "servos made"

        self.color = Color(self.tamp,
                           integrationTime=Color.INTEGRATION_TIME_101MS,
                           gain=Color.GAIN_1X)
        print "color sensor made"

        #start motors at 0
        self.right_motor.write(1,0)
        self.left_motor.write(1,0)

        # Set up webcam
        self.cap = cv2.VideoCapture(0)
        print "camera set up"

        self.bias = 0
        self.lastBlockReading = (True, 0, float("inf"))
        self.lastEncoderReading = 0
        self.distanceTraveled = 0
        print "Finished Setup"


    def loop(self):
        if (self.roundTimer.millis()<=150e3) and shouldContinue: #If there's still enough time to collect blocks
            #get frame from camera
            ret, self.frame = self.cap.read()

            if self.timer.millis() > self.refresh_rate:
                self.dt = self.timer.millis()/1000
                self.timer.reset()
                if self.calibrated:
                    """
                    get frame from camera
                    filter frames and get contours
                    see if there is a block
                    if there is block (not differentiating color): find angle and distance and drive toward block
                    if there is no block: turn clockwise 60 degrees
                    after turning six times without blocks, drive forward for a foot
                    """
                    #Color sensor
                    if self.checkColor() != -1 and self.colorTimer.millis()>=3000:
                        self.colorTimer.reset()
                        self.overallState= 2
                        self.driveBackState = 0
                        self.drivingToBlockState = 0
                        self.bias = 0
                        self.desired_theta = self.theta_est
                        self.driveBackState = 0
                    #Image Processing
                    #get all the contours
                    if (self.overallState not in [-1,2,3]) and ((self.drivingToBlockState<1 and self.overallState==1) or self.overallState==0):
                        if ret==True:
                            #filter red and green and get contours
                            self.height, self.width, channels = self.frame.shape
                            self.redF,self.greenF = self.filter_color(self.frame,self.height,self.width)
                            cntRed = self.filter_contours(self.redF,self.height,self.width,self.minFraction)
                            cntGreen = self.filter_contours(self.greenF,self.height,self.width,self.minFraction)
    #                        print "RedCnt:", len(cntRed)
    #                        print "GreenCnt:",len(cntGreen)
                            #sort contours by size big to small
                            allCnt = []
                            for cnt in cntRed:
                                allCnt.append(cnt)
                            for cnt in cntGreen:
                                allCnt.append(cnt)
#                            allCnt = np.concatenate(cntRed,cntGreen)
                            allCnt = sorted(allCnt,key = cv2.contourArea)
                            allCnt.reverse()
    #                        cv2.imshow("Original", frame)
    #                        cv2.imshow("RedFiltered", redF)
    #                        cv2.imshow("GreenFiltered", greenF)
    #                        if cv2.waitKey(1) & 0xFF == ord('q'):
    #                            pass
                        else:
                            #If there was no frame, allCnt is an empty list
                            print "No camera found"
                            allCnt = []
                            
                        #Check if there was a found block.
                        if len(allCnt)>0:
                            allCntNoButtons = []
                            for cnt in allCnt:
                                if not self.is_button(cnt,1.3):
                                    allCntNoButtons.append(cnt)
    #                                print "is a block"
                                else:
                                    pass
    #                                print "is a button"
                            if len(allCntNoButtons)>0:
    #                            print "target updated"
                                self.target = allCntNoButtons[0]
                                Cx, Cy = self.get_center_of_mass(self.target)
                                self.angle = self.get_angle(Cx, self.width)
                                self.desired_theta = self.theta_est + self.angle
                                if not self.overallState:                               
                                    self.overallState=1
                                    self.integral = 0
                            else:
                                #No non-button contours
                                self.target = None
                                self.overallState = 0
                                self.drivingToBlockState = 0
                        else:
                            #No contours
                            self.target = None
                            self.overallState = 0
                            self.drivingToBlockState = 0
                            
                    if self.numOurBlocks ==4 and (not self.overallState==3): #TODO set a goal and put in settings
                        self.blockPUS= 2
                        self.overallState = 2
                    #Perform action based on state
                    if self.overallState==0:
                        #Keep spinning
                        self.spin360()
                        
                    elif self.overallState == 1:
                        self.driveToBlock()
                        
                    elif self.overallState==2:
                        self.pickUpBlock()

                    elif self.overallState == 3:
                        self.driveBack()
                        
                    self.updateThetaEstNoEncoder()
                    self.calculatePower()
                    self.updateMotors()
                    self.deadReckon()
                                      

            # Janky autocalibration scheme
            if not self.calibrated and self.cali_timer.millis() > 3000:
                drift = self.gyro.val / (self.cali_timer.millis() / 1000.0)
                # Arbitrary calibration tolerance of 0.1 deg/sec
                if abs(drift) > 0.1:
                    self.calibration += drift
                    print "Calibration:", self.calibration
                    self.gyro.calibrate_bias(self.calibration)
                else:
                    print "Calibration complete:", self.calibration
                    self.calibrated = True
                self.gyro.reset_integration()
                self.cali_timer.reset()

            if self.statusTimer.millis() > 100:
                self.statusTimer.reset()
                print "desired theta: " + str(self.desired_theta)
                print "gyro theta: " + str(self.theta_est)
                print "encoder theta: " + str(self.theta_est_enc)
                print "LE:",self.left_motor_encoder.val
                print "RE:",self.right_motor_encoder.val
                print "desired distance: " + str(self.desired_dist)
                print "overallState:",self.overallState
                print "drivingToBlockState:",self.drivingToBlockState
                print "drive back state:",self.driveBackState
                print "block pick up state:",self.blockPUS
                print "bias",self.bias
                print "angle to block",self.angle

                print "P:",self.kP*self.last_diff
                print "I:",self.kI*self.integral
                print "D:",self.kD*self.derivative
                print "Power",self.power
                print "-------------------------------"
#        elif self.roundTimer.millis() > 150e3 and self.roundTimer.millis() <= 180e3:
#            #self.headBackToBase() These aren't created yet, but will need to be.
#            #self.releaseBlocks() These aren't created yet, but will need to be.
#            pass
        else:
            if not self.endGame:
                self.left_motor.write(1,0)
                self.right_motor.write(1,0)
                self.cap.release()
                self.endGame = True      

    def spin360(self):
        if abs(self.theta_est-self.desired_theta)<=10:
            self.desired_theta += 1
        self.bias = 0

    def driveToBlock(self):
        toleranceAng = 10
        changingDistance = 18.0
        extraDistance = 10.0
        
#        print self.angle
        
        if self.drivingToBlockState==0:

                
            #Stop relying on image when we are close enough to block
            distance = self.get_distance(self.target,self.height,self.width) #Inches
            #turn to the correct angle, facing the block without moving forward
            if abs(self.theta_est-self.desired_theta)>toleranceAng or distance<=changingDistance:
                self.bias = 0
                #when block is centered, move forward
            else:
                self.bias = self.init_motor_speed
                
                
            if distance<=changingDistance and abs(self.theta_est-self.desired_theta)<=toleranceAng:
                self.oldEncLeft = self.left_motor_encoder.val
                self.oldEncRight = self.right_motor_encoder.val
                self.drivingToBlockState = 1
                self.desired_dist = (changingDistance + extraDistance)
            
        elif self.drivingToBlockState==1:
            #stop calculating distance and drive the rest off distance + 5 inches
            #Dont update desired theta anymore. Trust we are centered enough and just drive straight            
            #turn to the correct angle, facing the block without moving forward
            if abs(self.theta_est-self.desired_theta)>toleranceAng:
                self.bias = 0
                #when block is centered, move forward
            else:
                self.bias = self.init_motor_speed
            #Once we have driven far enough go into the exit state
            if self.desired_dist <= 0:
                self.drivingToBlockState = 0
                self.overallState = 2
                self.target = None
                self.bias = 0
                self.desired_theta = self.theta_est

    def pickUpBlock(self):
        #drive forward a small distance then check block color and 
        toleranceAng = 10
        smallDistance = 5.0
        if self.blockPUS==-2:
            #drive forward a couple inches
            self.desired_dist = smallDistance
            self.desired_theta = self.theta_est
            self.blockPUS += 1
            
        elif self.blockPUS==-1:
            #drive forward a couple inches
            if abs(self.theta_est-self.desired_theta)>toleranceAng:
                self.bias = 0
                #when block is centered, move forward
            else:
                self.bias = self.init_motor_speed
            if self.desired_dist<=0:
                #TODO
                self.blockPUS = 0
                self.bias = 0
                self.desired_theta = self.theta_est
#                self.blockPUS = self.checkColor()
                if self.blockPUS==-1:
                    #No block anymore recalculate path to get back to where we were going
                    self.overallState==0
        elif self.blockPUS==0:
            #pick up our block
            self.pickUpOurs()
            
        elif self.blockPUS==1:
            #pick up opponent
            self.pickUpOpponent()
            
        elif self.blockPUS==2:
            #drop our stack
            self.dropStack()
            

    def updateThetaEstNoEncoder(self):
        #Only use gyroscope to estimate current angle
        self.theta_est = self.gyro.val

    def updateThetaEstEncoder(self):

        self.theta_est_enc = float(self.left_motor_encoder.val - self.right_motor_encoder.val)*self.wheel_radius*9.0/(80.0*self.axel_length)

    def calculatePower(self):
        #Use PID and update motors
        diff = self.desired_theta - self.theta_est
#        self.integral = self.integral + diff*self.dt
#        self.derivative = (diff - self.last_diff)/self.dt
        integ = self.integral + diff*self.dt
        try:
            alpha = self.maxCorrection/self.kI
        except ZeroDivisionError:
            alpha = 0
        if integ < 0:
            self.integral = max(integ, -alpha)
        else:
            self.integral = min(integ, alpha)
            
        deriv = (diff - self.last_diff)/self.dt
        try:
            beta = self.maxCorrection/self.kD
        except ZeroDivisionError:
            beta = 0
        if deriv < 0:
            self.derivative = max(deriv, -beta)
        else:
            self.derivative = min(deriv, beta)
        
        self.power = self.kP*diff + self.kI*self.integral + self.kD*self.derivative
        self.last_diff = diff

    def updateMotors(self):
        #Prevent a power that will try to set the wheels above or below possible values
        if ((self.bias + self.power)>255) or ((self.bias - self.power)<-255):
            self.power = 255 - abs(self.bias)
        elif ((self.bias + self.power)<-255) or ((self.bias - self.power)>255):
            self.power = -255 + abs(self.bias)

        #Stop power from being too high so the change in wheel speed is not too drastic
        if self.power>self.maxCorrection:
            self.power = self.maxCorrection
        elif self.power<-self.maxCorrection:
            self.power = -self.maxCorrection

        if abs(self.last_diff) >= 10:
            self.bias = 0
        #Set motor values
        self.motorval_left = self.bias + self.power + 15.0
        self.motorval_right = self.bias - self.power - 15.0
        self.left_motor.write(self.motorval_left>0, abs(self.motorval_left))
        self.right_motor.write(self.motorval_right>0, abs(self.motorval_right))

    def is_button(self, cnt, threshold = 1.3):
        """
        cnt: a contour, threshold: a float, for the minimum ratio needed for a contour to be a circle
        return: True if the contour is a circle, False otherwise
        """
#        return False
        areaOld = cv2.contourArea(cnt)
        epsilon= 0.05 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        a = cv2.contourArea(approx)
        
        try:
            ratio = float(areaOld)/a
        except ZeroDivisionError:
            print "ZDE"
            return True
        print ratio
        return ratio > threshold


    def get_center_of_mass(self, cnt):
        """
        takes in a contour and finds the center of mass enclosed in the contour
        returns Cx, Cy
        to color it black: frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
        """
        M = cv2.moments(cnt)
#        print "m00",M['m00']
#        print "m10",M['m10']
        if M['m00']!=0:
            Cx = int(M['m10']/M['m00'])
            Cy = int(M['m01']/M['m00'])
#            print "Cx",Cx
            return Cx, Cy
        else:
            return 0,0

    def get_angle(self, Cx, width):
        """
        input: x value of center of mass, width of the frame
        return the angle in front of the camera, 0 is in the middle, positive to the right, and negative to the left
        """
        return (60.0*float(Cx)/width - 30.0)
        
    def get_distance(self, cnt,height,width):
        
        v = (height*width)/(10*cv2.contourArea(cnt))
        dist = -0.0429*(v**2) + 2.3863*v + 12.291
        return dist

    def filter_contours(self, frame,height,width,ratio = 0.01):
        
        imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,50,255,0)
        if ret==False:
            return []
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        minArea = ratio*height*width
        realContours = []
        for cnt in contours:
            cntArea = cv2.contourArea(cnt)
            if cntArea>=minArea:
                realContours.append(cnt)
            else:
                frame = cv2.drawContours(frame,[cnt],0,[0,0,0],-1)
#        sort realcontours by area    
#        sortedCnts = sorted(realContours,key=cv2.contourArea)
#        sortedCnts.reverse()
#        return sortedCnts
        return realContours
        

    def filter_color(self, frame,height,width):
        
        filteredFrameRed = np.zeros((height,width,3),np.uint8)
        filteredFrameGreen = np.zeros((height,width,3),np.uint8)

        b,g,r = cv2.split(frame)
        filteredFrameRed[((r>(self.rg*g)) & (r>(self.rb*b)))] = [0,0,255]
        filteredFrameGreen[((g>(self.gr*r)) & (g>(self.gb*b)))] = [0,255,0]
        
        return filteredFrameRed,filteredFrameGreen

    def deadReckon(self):
            leftEnc = self.left_motor_encoder.val
            rightEnc = self.right_motor_encoder.val
            deltaL = (leftEnc - self.oldEncLeft) * self.wheel_circumference/3200.0
            deltaR = (rightEnc - self.oldEncRight) * self.wheel_circumference/3200.0
            if (abs(deltaL - deltaR)) <= 1e-6:
                dx = deltaL * np.cos(self.heading)
                dy = deltaR * np.sin(self.heading)
    #            self.heading = np.radians(-self.gyro.val)

            else:
                r = self.axel_length * (deltaL + deltaR) / (2 * (deltaR - deltaL))
                wd = (deltaR - deltaL) / self.axel_length
                
                dx = r* np.sin(self.heading + wd) - r*np.sin(self.heading)
                dy = -r* np.cos(self.heading + wd) + r*np.cos(self.heading)
                self.heading += wd
    #            self.heading = np.radians(-self.gyro.val)

            self.x = self.x +  dx
            self.y = self.y +  dy
            self.distTravelled += np.sqrt(dy**2 + dx**2)
            self.desired_dist -= np.sqrt(dy**2 + dx**2)
            self.oldEncLeft = leftEnc
            self.oldEncRight = rightEnc
    
    def pickUpOurs(self):
        if self.collectTimer.millis()>=self.servoDelayOurs:
            self.collectTimer.reset()
            if self.ourBlockPUS==0:
                print "taking grabber out"
                self.grabber.write(self.grabber_angles[1]) #take grabber out
                self.ourBlockPUS=1
                
            elif self.ourBlockPUS==1:
                print "lowering elevator"
                self.elevator.write(self.ele_angles[1]) #drop elevator
                self.ourBlockPUS=2
                
            elif self.ourBlockPUS==2:
                print "inserting grabber"
                self.grabber.write(self.grabber_angles[0]) #put grabber in
                self.ourBlockPUS=3
            
            elif self.ourBlockPUS==3:
                print "raising elevator"
                self.elevator.write(self.ele_angles[0]) #raise elevator
                self.ourBlockPUS=0
                self.blockPUS=-2
                self.numOurBlocks += 1
                self.overallState = 0
                self.bias = 0
                self.desired_theta = self.theta_est
            
    def pickUpOpponent(self):
        #Wait for 500ms the first time through to allow skewer to fully insert then only wait 100ms each time.
        if ((self.collectTimer.millis()>=self.servoDelayOpp) and (self.angleToWrite != self.skew_angles[0]))\
            or ((self.collectTimer.millis()>=self.servoDelayOurs) and (self.angleToWrite == self.skew_angles[0])):
            self.collectTimer.reset()
            #Extend to collect block
            if self.oppBlockPUS==0:
                self.skewer.write(self.skew_angles[0])
                self.oppBlockPUS = 1
            
            elif self.oppBlockPUS==1:
                #Slowly increment until it is fully retracted
                if self.angleToWrite < self.skew_angles[1]:
                    self.angleToWrite += 1
                    self.skewer.write(min(self.angleToWrite,self.skew_angles[1]))
                #When fully retracted revert back to waiting for block state
                if self.angleToWrite >= self.skew_angles[1]:
                    self.angleToWrite = self.skew_angles[0]
                    self.blockPUS = -2
                    self.oppBlockPUS = 0
                    self.numOppBlocks += 1
                    self.overallState = 0
                    self.bias = 0
                    self.desired_theta = self.theta_est
                    
                
    def dropStack(self):
        if self.collectTimer.millis()>=self.servoDelayDropStack:
            self.collectTimer.reset()
            if self.dropState==0:
                self.grabber.write(self.grabber_angles[1]) #take grabber out
                self.dropState=1
                
            elif self.dropState==1:
#                self.elevator.write(self.ele_angles[1]) #drop elevator
                self.door.write(self.door_angles[1]) #open door
                self.dropState=2
                
            elif self.dropState==2:
#                self.elevator.write(self.ele_angles[0]) #raise elevator
                self.dropState=0
                self.blockPUS=-2
                self.overallState = 3
                self.bias = 0
                self.desired_theta = self.theta_est
    
    def driveBack(self):
        if self.driveBackState == 0:
            self.desired_dist = 10
            self.desired_theta =  self.theta_est
            self.driveBackState = 1
        elif self.driveBackState == 1:
            if abs(self.desired_theta - self.theta_est) > 5: # when not centered, recenter
                self.bias = 0
            else:
                self.bias = - self.init_motor_speed
            if self.desired_dist <= 0:
                self.bias = 0
                self.desired_theta = self.theta_est
                self.overallState = -1
                self.driveBackState = -1
        else:
            pass

    def checkColor(self):
        if (self.color.r>self.colorSenserg*self.color.g) and (self.color.r>self.colorSenserb*self.color.b):
            blockColor = "red"
#            print "red block"
        elif (self.color.g>self.colorSensegr*self.color.r) and (self.color.g>self.colorSensegb*self.color.b):
            blockColor = "green"
#            print "green block"
        else:
            blockColor = "None"
#            print "no block"
            
        if blockColor=="None":
            return -1
        elif blockColor == self.ourColor:
            return 0
        else:
            return 1
    

def playSound():
    fileNames = ["C:/Users/arinz/OneDrive/Documents/GitHub/team1/Sound Practice/DorsiFlex.wav","C:/Users/arinz/OneDrive/Documents/GitHub/team1/Sound Practice/shit.wav","C:/Users/arinz/OneDrive/Documents/GitHub/team1/Sound Practice/OurClap.wav", "C:/Users/arinz/OneDrive/Documents/GitHub/team1/Sound Practice/CheerShort.wav"]
    startTime = time.time()
    originalTime = time.time()
    shouldPlayCheer = True
    while shouldContinue:
        #Stop after 3 minutes
        if time.time()-originalTime>180:
            break
        try:
            state = sketch.overallState
            if sketch.roundTimer.millis()>=150e3:
                break
        except (NameError,AttributeError):
#            print "no sketch yet"
            state = 0
            
        if state==0:
            #Dorsiflex every second
            count = 0
            if random.random()<0.8:
                winsound.SND_FILENAME = fileNames[0]
            else:
                winsound.SND_FILENAME = fileNames[1]
            desiredPeriod = 1.0
            
        elif state==1:
            shouldPlayCheer = True
            #Slow clap
            
            winsound.SND_FILENAME = fileNames[2]
            desiredPeriod = (5.0/3.0)*math.exp(-0.3*count) + (1.0/5.0)
            
        elif state==2 and shouldPlayCheer:
            shouldPlayCheer = False
            #Cheer
            count = 0
            winsound.SND_FILENAME = fileNames[3]
            desiredPeriod = 2.0
        
        #Playsound and reset timer
        if time.time() - startTime >= desiredPeriod:
            count += 1
            winsound.PlaySound(winsound.SND_FILENAME,winsound.SND_ASYNC)
            startTime = time.time()

def showVideoStream():
    try:
        blankWindow = np.zeros((sketch.height,sketch.width,3),np.uint8)
    except (NameError,AttributeError):
        blankWindow = np.zeros((480,640,3),np.uint8)
        
    while shouldContinue:
        #Get frames
        try:
            frame = sketch.frame
        except (NameError,AttributeError):
            frame = blankWindow
        try:
            redFrame = sketch.redF
        except (NameError,AttributeError):
            redFrame = blankWindow
        try:
            greenFrame = sketch.greenF
        except (NameError,AttributeError):
            greenFrame = blankWindow
            
        # Display the resulting frame
        cv2.imshow("Original", frame)
        cv2.imshow("Red Filtered", redFrame)
        cv2.imshow("Green Filtered", greenFrame)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # When everything done, release the capture
#    sketch.cap.release()
    cv2.destroyAllWindows()
    
    

if __name__ == "__main__":
    shouldContinue=True
    
    sketch = BlockSearchV1(1, -0.00001, 100)
    sketchThread = threading.Thread(name="blocksearch",target=sketch.run)
    print "Sketch Made"
#    sketch.run()
#    soundThread = threading.Thread(name="Sounds",target=playSound)
    videoThread = threading.Thread(name="VideoCam",target=showVideoStream)
    print "Starting Threads"
    sketchThread.start()
#    soundThread.start()
    videoThread.start()
    try:
        while True:
            pass
    except(KeyboardInterrupt, SystemExit):
        shouldContinue = False     
    print "exited loop"
    
