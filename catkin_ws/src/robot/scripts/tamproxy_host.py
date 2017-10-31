#!/usr/bin/env python

# add the path to TAMProxy to sys
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost')
import Settings
from tamproxy import Sketch, SyncedSketch, Timer, TAMProxy
from tamproxy.devices import Gyro, Encoder, Motor, Servo, Color
#from Settings import *

# import ROS stuff
import roslib
import rospy
from robot.msg import GyroMsg, EncoderMsg, MotorMsg, ServoMsg, ColorSensorMsg


class tamProxyHost(object):
    def __init__(self):
        # create a tamproxy instance that will be passed around to the different sensors
        self.tamp = TAMProxy()

        # set continuous readings so that sensor data is automatically requested
        self.tamp.pf.pc.set_continuous_enabled(True)

        # create gyro
        self.gyro = Gyro(self.tamp, Settings.ss_pin)

        # create color sensor
        self.color = Color(self.tamp,
                           integrationTime=Color.INTEGRATION_TIME_101MS,
                           gain=Color.GAIN_1X)
        self.color_sensor_msg = ColorSensorMsg()


        # create encoders
        self.left_encoder = Encoder(self.tamp, Settings.left_motor_encoder_pins[0], Settings.left_motor_encoder_pins[1])
        self.right_encoder = Encoder(self.tamp, Settings.right_motor_encoder_pins[0], Settings.right_motor_encoder_pins[1])

        # create motors
        self.left_motor = Motor(self.tamp, Settings.left_motor_dir_pin, Settings.left_motor_pwm_pin)
        self.right_motor = Motor(self.tamp, Settings.right_motor_dir_pin, Settings.right_motor_pwm_pin)

        self.left_motor_pwm = 0
        self.right_motor_pwm = 0
        self.left_motor_dir = True
        self.right_motor_dir = True

        # create 4 servos
        self.grabber_pos = Settings.grabber_angles[1] #grabber starts out
        self.elevator_pos = Settings.ele_angles[0] #elevator starts up
        self.skewer_pos = Settings.skew_angles[1] #skewer starts out
        self.door_pos = Settings.door_angles[0] # door starts closed

        #subscribe to the servo topic
        self.servo_sub = rospy.Subscriber('robot/servos', ServoMsg, self.updateServoPositions)

        self.grabber = Servo(self.tamp, Settings.GRABBER_PIN)
        #self.grabber.write(self.grabber_pos)
        self.grabber.write(8)

        #rospy.sleep(2.4)
        self.elevator = Servo(self.tamp, Settings.ELE_PIN)
        self.elevator.write(self.elevator_pos)
        #self.elevator.write(125)
        
        #rospy.sleep(2.4)
        self.skewer = Servo(self.tamp, Settings.SKEW_PIN)
        self.skewer.write(self.skewer_pos)
        #self.skewer.write(54)

        #rospy.sleep(2.4)
        self.door = Servo(self.tamp, Settings.DOOR_PIN)
        self.door.write(self.door_pos)
        #self.door.write(130)

        # create sensor publishers
        self.gyro_pub = rospy.Publisher('robot/gyro', GyroMsg, queue_size=10)
        self.encoder_pub = rospy.Publisher('robot/encoders', EncoderMsg, queue_size=10)
        self.color_pub = rospy.Publisher('robot/color_sensor', ColorSensorMsg, queue_size=1)

        # publish everything at 20 Hz
        self.rate = rospy.Rate(20.0)
        self.gyro_msg = GyroMsg()
        self.encoder_msg = EncoderMsg()


        # subscribe to the motor topic
        self.motor_sub = rospy.Subscriber('robot/motors', MotorMsg, self.updateMotorCommands)

        print "SAFE TO TURN ON SERVOS"
        while not rospy.is_shutdown():
            # write positions to the servos
            self.grabber.write(self.grabber_pos)
            self.elevator.write(self.elevator_pos)
            self.skewer.write(self.skewer_pos)
            self.door.write(self.door_pos)


            # self.grabber.write(8)
            # self.elevator.write(125)
            # self.skewer.write(54)
            # self.door.write(130)

            # get latest data from the gyro and publish
            self.gyro_msg.angle = self.gyro.val
            self.gyro_msg.angular_vel = self.gyro.angular_vel
            self.gyro_pub.publish(self.gyro_msg)

            # publish the latest color sensor values
            self.color_sensor_msg.r, self.color_sensor_msg.g, self.color_sensor_msg.b, \
            self.color_sensor_msg.c = self.color.r, self.color.g, self.color.b, self.color.c
            self.color_pub.publish(self.color_sensor_msg)


            # get latest data from the encoders and publish
            self.encoder_msg.left_encoder_val = self.left_encoder.val
            self.encoder_msg.right_encoder_val = self.right_encoder.val
            self.encoder_pub.publish(self.encoder_msg)

            # write commands to the motors
            self.left_motor.write(self.left_motor_dir, self.left_motor_pwm)
            self.right_motor.write(self.right_motor_dir, self.right_motor_pwm)

            

            self.rate.sleep()

        # stop the TAMProxy instance when done!
        self.tamp.stop()


    def updateMotorCommands(self, data):
        """
        Every time a new motor message is received, we update the local motor values.
        """
        self.left_motor_dir = data.left_motor_dir
        self.left_motor_pwm = data.left_motor_pwm
        self.right_motor_dir = data.right_motor_dir
        self.right_motor_pwm = data.right_motor_pwm

    def updateServoPositions(self, data):
        """
        Every time a new servo message is received, we update the local servo values.
        """
        self.grabber_pos = data.grabber_servo_pos
        self.elevator_pos = data.elevator_servo_pos
        self.skewer_pos = data.skewer_servo_pos
        self.door_pos = data.door_servo_pos

    


if __name__ == "__main__":
    rospy.init_node('tamproxy_host')
    try:
        host = tamProxyHost()
    except rospy.ROSInterruptException: pass




