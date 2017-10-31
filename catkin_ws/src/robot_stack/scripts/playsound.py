
#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings as S

import numpy as np, cv2
import roslib
import rospy
from robot_stack.msg import VisionMsg, DriveMsg, GyroMsg, EncoderMsg, StateMsg, PickUpMsg
import pyaudio
import wave
import time
import random

class PlaySound(object):
    """
    Based on the current vision msg, sends drive commands to navigate to a block in the robot's FOV.
    """
    def __init__(self):

        self.stateSub = rospy.Subscriber('robot/state', StateMsg, self.updateState)
        self.state = "STOP"
        self.rate = rospy.Rate(20)
        self.dir = '/home/maslab/Code/team1/catkin_ws/src/robot_stack/audio/' #directory name
        self.clap = 'clap2.wav' #clap
        self.cheer = 'cheer.wav'
        self.dorsiflex = 'DF.wav'
        self.hwatg = 'Hwatg.wav'
        self.mess = ['Duel meets.wav', 'Jab plant push.wav', 'push against the curve.wav', "Whatever you're doing, don't.wav", 'Where the hell has that been.wav']

        # runs at 20 Hz
        while not rospy.is_shutdown():

            # see if we aren't allowed to drive due to the current state
            if (self.state == 'COLL_OUR') or (self.state == 'COLL_OPP'):
                self.playMessage(self.dir + self.cheer)
            elif (self.state == 'STOP') or (self.state == 'RELEASE'):
                # stop the robot
                self.playMessage(self.dir + self.cheer)
                self.playMessage(self.dir + self.hwatg)
                

            elif self.state == 'REVERSE':
                self.playMessage(self.dir + self.dorsiflex)

                # this pick up state tells the state controller that we're DONE

            elif self.state == 'NAV_BLOCK': # allowed to drive
                self.slowClap(self.dir + self.clap)

            elif self.state == 'SCAN':
                # have the robot slowly spin CW
                # play a random message
                self.playMessage(self.dir + self.mess[random.randint(0, len(self.mess))]

            elif self.state == 'UNSTICK':
                self.playMessage(self.dir + self.cheer)

            else:
                pass

            self.rate.sleep()



    def updateState(self, data):
        """
        Listens for state changes from the state controller.
        """
        self.state = data.state

    def slowClap(fileName):

        CHUNK = 1024
        wf = wave.open(fileName, 'rb')

        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

        start = time.time()
        data = wf.readframes(CHUNK)
        sound = []

        while data != '':
            sound.append(data)
            data = wf.readframes(CHUNK)


        wait =  2.0
        s = time.time()
        num = 0
        while True:
            if (time.time() - s) >= wait + 0.2:
                for d in sound:
                    stream.write(d)
                    if time.time() - s >= wait + wait* 0.8 + 0.2:
                        break
                s = time.time()
                wait = wait * 0.8
                num += 1
            if num >= 25:
                break

        stream.stop_stream()
        stream.close()
        p.terminate()


    def playMessage(fileName, time = 5):
        CHUNK = 1024
        wf = wave.open(fileName, 'rb'
        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

        start = time.time()
        data = wf.readframes(CHUNK)
        sound = []

        while data != '':
        #    stream.write(data)
            sound.append(data)
            data = wf.readframes(CHUNK)

        for d in sound:
            stream.write(d)
            if time.time() - start > = time:
                break
            
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == '__main__':
    rospy.init_node('play_sound')
    try:
        playSound = playSound()
    except rospy.ROSInterruptException: pass
