#settings


# GYRO PINS #
# GND -> GND RAIL
# PDD -> +3.3V
# MOSI -> 11
# MISO -> 12
# CS -> 10
# CK -> 13

# ENCODER PINS #
# RIGHT: Yellow -> 6, White -> 7
# LEFT: White -> 8, Yellow -> 9

import numpy
#motor pins:
# note: a purple wire should go into 23!
left_motor_dir_pin, left_motor_pwm_pin = 21,20
right_motor_dir_pin, right_motor_pwm_pin = 23,22
#encoder pins
left_motor_encoder_pins = 8,9
right_motor_encoder_pins = 6,7
#gyroscope pin
ss_pin = 10 #CS on the gyro
#set the initial motor speed
init_motor_speed = 100
#angle we want to turn to
desired_theta = 0

#physical constants
#shouldGoAgain = True
theta_est = 0.0
theta_est_enc = 0.0

#robot parameters
#axel_length = 8.25 #distance between two wheels in inches
#wheel_radius = 1.9375 #radius of wheels in inches
axel_length = 15.125 #distance between two wheels in inches, approximate
wheel_radius = 1.425 #radius of orange wheels in inches, but specs said 1.4375

axel_length_meters = axel_length / 39.3701
wheel_radius_meters = wheel_radius / 39.3701
wheel_circumference_meters = 2*wheel_radius_meters*numpy.pi

wheel_circumference = 2*wheel_radius*numpy.pi
counts_per_rotation = 3200.0 #from the motor ecoder specs
refresh_rate = 10

#PID constants
kU = 22.5
tU = 1.5

kP = 0.6*kU
kI = kP*2.0/tU
kD = kP*tU/80.0

#kP = 2#0.6*kU
#kI = 1#tU/2
#kD = 0.110#tU/8.0
maxCorrection = 70

#colors
rg = 1.7 # red to green ratio
rb = 1.7
gr = 1.1
gb = 1.1
br = 1.5
bg = 1.5

#gyro drift: 
drift_m = 0.2103 # slope
drift_b = -0.667 # intercept

GRABBER_PIN = 2
grabber_angles = [27, 8] #in and out

ELE_PIN = 3 
ele_angles = [125, 0] # up and down

SKEW_PIN = 4
skew_angles = [0, 54]#in and out

DOOR_PIN = 5
door_angles = [130,10] #10 is not set in stone, [closed,open]

blockPUS = -1 #-1 is no block, 0 is our block, 1 is opponent block
ourBlockPUS = 0
oppBlockPUS = 0
dropState = 0

servoDelayOurs = 500 #millisec
servoDelayOpp = 10 #millisec

colorSenserg = 1.8 # red to green ratio
colorSenserb = 1.8
colorSensegr = 1.2
colorSensegb = 1.25

ourColor = "red"
#ourColor = "green"
