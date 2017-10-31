#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings as S

import roslib
import rospy
import time
from robot_stack.msg import VisionMsg, StateMsg, PickUpMsg



class stateController(object):
	"""
	SCAN: Rotating slowly in search of nearby blocks.
	NAV_BLOCK: Navigating towards a FOV block using webcam.
	EXTRA: Going a small amount of extra distance to make sure the block gets inside.
	POS_BLOCK: Go forward 2 inches to make sure block is positioned correctly (happens when color sensor first sees a block)
	COLL_OUR: In the process of collecting our color block.
	COLL_OPP: In the process of collecting an opp. block.
	NAV_BUTTON: Navigating towards the button (in view)
	NAV_BASE: Navigating towards our base (with path planning)
	UNSTICK: Trying to get away from a wall that we're stuck on.
	RELEASE: Releasing our stack by opening the door and slowly backing away.
	REVERSE: Backing away from the stack at the end of the match.
	STOP: Doing nothing. We start and end the match in the STOP state.
	"""
	def __init__(self):
		self.rate = rospy.Rate(20)
		self.statePub = rospy.Publisher('/robot/state', StateMsg, queue_size=10)
		self.stateMsg = StateMsg('SCAN') # don't move at first
		self.matchStartTime = time.time()
		self.elapsedTime = 0
		self.pickUpState = 'None'
		self.numOurBlocks = 0
		self.numOppBlocks = 0
		self.maxOurBlocks = 10
		self.maxOppBlocks = 6
		self.visionState = 'None'
		self.releaseTime = 165 # stop and release the stack at this point (sec)

		# for each state, we have an associated timeout
		# after being in this state for the timeout duration, we call a handleTimeout() function
		# which puts us in another state
		self.stateTimeouts = {
			'SCAN':15.0, 
			'NAV_BLOCK':8.0, 
			'UNSTICK':8.0, 
			'NAV_BUTTON':8.0,
			'NAV_BASE':30.0,
			'EXTRA': 5.0, 
			'REVERSE':8.0, 
			'STOP':float('inf'),
			'DROP': 10.0,
			'COLL_OUR':10.0,
			'COLL_OPP':10.0,
			'RELEASE':10.0,
			'POS_BLOCK':4.0
		}

		self.lastState = self.stateMsg.state
		self.lastStateTime = time.time()

		# subscribe to other topics that will have control over our state
		#self.colorSub = rospy.Subscriber('robot/color', ColorSensorMsg, self.updateColorSensorState)
		self.visionSub = rospy.Subscriber('robot/vision', VisionMsg, self.updateVisionState)
		self.pickUpSub = rospy.Subscriber('robot/pick_up_state', PickUpMsg, self.updatePickUpState)
		
		# runs at 10 Hz
		while not rospy.is_shutdown():


			# get the current duration of the match (sec)
			self.elapsedTime = time.time() - self.matchStartTime

			# TIME LEFT IN MATCH, AND HAVE NOT REACHED BLOCK LIMITS
			if self.elapsedTime < self.releaseTime and self.numOurBlocks < self.maxOurBlocks and self.numOppBlocks < self.maxOppBlocks:

				# check if we've reached the timeout for our current state in the timeout dictionary
				self.stateTimer = (time.time() - self.lastStateTime)
				if (self.stateTimer > self.stateTimeouts[self.stateMsg.state]):
					self.handleTimeout(self.stateMsg.state)

				else:
					# if we're in the process of picking up
					if self.pickUpState=="OUR":
						self.stateMsg.state = "COLL_OUR"

					elif self.pickUpState=="OPP":
						self.stateMsg.state = "COLL_OPP"

					elif self.pickUpState == 'EXTRA':
						self.stateMsg.state = 'EXTRA'

					elif self.pickUpState == 'POS_BLOCK':
						self.stateMsg.state = 'POS_BLOCK'


					else: # not picking up, allow states that require driving
						if self.visionState == "BLOCK":
							self.stateMsg.state = "NAV_BLOCK"

						# this means we finished unstick and want to go back to scan
						elif self.pickUpState == -4 and self.stateMsg.state == 'UNSTICK':
							self.stateMsg.state = "SCAN"
		
						# TODO: add in state for button
						


			# REACHED MATCH END TIME, GO THROUGH FINISHING SEQUENCE
			else:
				print "Reached match time, dropping"
				print self.pickUpState
				# if we've finished dropping the stack, reverse
				if self.pickUpState == 'DROP':
					self.stateMsg.state = "REVERSE"

				elif self.pickUpState == 'DONE':
					self.stateMsg.state = 'STOP'
				
				else:
					self.stateMsg.state = "RELEASE"

			

			# check if we changed state, and if so, reset the lastStateTime to now
			print "ELAPSED: %.2f | STATE: %s %.2f | VISION: %s | PICKUP: %s | " \
				% (self.elapsedTime, self.stateMsg.state, self.stateTimer, self.visionState, self.pickUpState)

			# if we change states, reset the state timer
			if self.stateMsg.state != self.lastState: 
				self.lastStateTime = time.time()
				self.stateTimer = 0
				self.lastState = self.stateMsg.state

			# publish the current state
			self.statePub.publish(self.stateMsg)

			self.rate.sleep()


	def handleTimeout(self, timeoutState):
		"""
		Whenever we hit a timeout, this function is called to determine which
		state we enter next.

		timeoutState: the state that we reached a timeout in
		"""
		print "HIT TIMEOUT OF %s" % timeoutState
		if timeoutState == 'SCAN':
			self.stateMsg.state = 'UNSTICK'

		elif timeoutState == 'EXTRA':
			self.stateMsg.state = 'UNSTICK'

		elif timeoutState == 'NAV_BLOCK':
			self.stateMsg.state = 'UNSTICK'

		elif timeoutState == 'UNSTICK':
			self.stateMsg.state = 'EXTRA'

		elif timeoutState == 'REVERSE':
			self.stateMsg.state = 'STOP'

		elif timeoutState == 'NAV_BUTTON':
			self.stateMsg.state = 'UNSTICK'

		elif timeoutState == 'POS_BLOCK':
			self.stateMsg.state = 'UNSTICK'

		else: # by default, go back to scan
			self.stateMsg.state = 'SCAN'


	def updatePickUpState(self, data):
		"""
		Whenever the pick_up node or the target_blocks node change the pick up state,
		this function gets called.
		"""
		if data.pick_up_state == 10:
			self.pickUpState = "DONE"
		if data.pick_up_state == 1:
			self.pickUpState = "OPP"
		elif data.pick_up_state == 0:
			self.pickUpState = "OUR"
		elif data.pick_up_state == 2:
			self.pickUpState = 'DROP'
		elif data.pick_up_state == -2:
			self.pickUpState = 'EXTRA'
		elif data.pick_up_state == -1:
			self.pickUpState = "None"
		elif data.pick_up_state == -3:
			self.pickUpState = 'POS_BLOCK'
		elif data.pick_up_state == -4:
			self.pickUpState = 'UNSTICK_COMPLETE'
			# update number of blocks we think we have
		self.numOurBlocks, self.numOppBlocks = data.num_our_blocks, data.num_opp_blocks


	def updateVisionState(self, data):
		if self.stateMsg.state == 'UNSTICK':
			self.visionState = 'None'
		# elif data.button_visible:
		# 	self.visionState = "BUTTON"
		elif data.block_visible:
			self.visionState = "BLOCK"
		else:
			self.visionState = "None"



if __name__ == '__main__':
    rospy.init_node('state_controller')
    try:
        stateController = stateController()
    except rospy.ROSInterruptException: pass
