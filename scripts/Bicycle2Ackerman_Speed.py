#!/usr/bin/env python
import rospy
from math import *
from std_msgs.msg import Float64
from MPC_Sim.msg import BicycleControl_msg

class Bicycle2Ackerman(object):
    def __init__(self,pubFrStL,pubFrStR,pubFrDrR,pubFrDrL,pubRrDrR,pubRrDrL):
        self._pubFrStR = pubFrStR
        self._pubFrStL = pubFrStL
        self._pubFrDrR = pubFrDrR
        self._pubFrDrL = pubFrDrL
        self._pubRrDrR = pubRrDrR
        self._pubRrDrL = pubRrDrL
        self._wheel_base = 2.600  # l in m
        self._track_width = 1.500 # w in m
        self._a = self._wheel_base /2
        self._deltaBi = 0
        self._FrSpin = 0
        self._RrSpin = 0
        self._delaOut = 0 
        self._delaIn = 0
        self._FastFrSpin = 0
        self._SlowFrSpin = 0
        self._FastRrSpin = 0
        self._SlowRrSpin = 0
        self._R1 = 0
        self._r = 0.2
    def AckermanCondition(self): # CW: Positive CCW: Negative
        if(self._deltaBi != 0):
            self._R1 = self._wheel_base*cot(abs(self._deltaBi))
            self._delaOut = atan(self._wheel_base/(self._R1 + self._track_width /2))
            self._delaIn = atan(self._wheel_base/(self._R1 - self._track_width /2))
        else:
            self._delaOut = 0
            self._delaIn = 0
    def DifferentialSpinning(self):
        if(self._deltaBi != 0):
            self._R_Outer_Fr = sqrt((self._wheel_base * self._wheel_base) + (self._R1 + self._track_width /2) * (self._R1 + self._track_width /2))
            self._R_Inner_Fr = sqrt((self._wheel_base * self._wheel_base) + (self._R1 - self._track_width /2) * (self._R1 - self._track_width /2))
            self._R_Outer_Rr = (self._R1 + self._track_width /2)
            self._R_Inner_Rr = (self._R1 - self._track_width /2)
            self._R_Bicycle_Fr = sqrt((self._wheel_base * self._wheel_base) + (self._R1 * self._R1))

            self._FastFrSpin = self._FrSpin * self._R_Inner_Fr / self._R_Bicycle_Fr 
            self._SlowFrSpin = self._FrSpin * self._R_Outer_Fr / self._R_Bicycle_Fr
            self._FastRrSpin = self._RrSpin * self._R_Inner_Rr / self._R1
            self._SlowRrSpin = self._RrSpin * self._R_Outer_Rr / self._R1
        else:
            self._FastFrSpin = self._FrSpin
            self._SlowFrSpin = self._FrSpin
            self._FastRrSpin = self._RrSpin
            self._SlowRrSpin = self._RrSpin

    def CheckAckerman(self):
        if(self._deltaBi != 0):
            rospy.loginfo('=========================')
            rospy.loginfo('R_Outer_Fr: {}'.format(self._R_Outer_Fr))
            rospy.loginfo('R_Inner_Fr: {}'.format(self._R_Inner_Fr))
            rospy.loginfo('R_Inner_Rr: {}'.format(self._R_Inner_Rr))
            rospy.loginfo('R_Outer_Rr: {}'.format(self._R_Outer_Rr))
            rospy.loginfo('=========================')
            rospy.loginfo('_FastFrSpin: {}'.format(self._FastFrSpin))
            rospy.loginfo('_SlowFrSpin: {}'.format(self._SlowFrSpin))
            rospy.loginfo('_FastRrSpin: {}'.format(self._FastRrSpin))
            rospy.loginfo('_SlowRrSpin: {}'.format(self._SlowRrSpin))
            rospy.loginfo('=========================')
            rospy.loginfo('w_Outer_Fr: {}'.format(self._SlowFrSpin / self._R_Outer_Fr))
            rospy.loginfo('w_Inner_Fr: {}'.format(self._FastFrSpin / self._R_Inner_Fr))
            rospy.loginfo('w_Inner_Rr: {}'.format(self._FastRrSpin / self._R_Inner_Rr))
            rospy.loginfo('w_Outer_Rr: {}'.format(self._SlowRrSpin / self._R_Outer_Rr))
        
    def EnforceConstraints(self):
        if(self._delaOut > pi/4):
            self._delaOut = pi/4
        if(self._delaIn > pi/4):
            self._delaIn = pi/4

    def Control_callback(self,Msg):
        self._deltaBi = -Msg.deltaBi
        self._FrSpin = Msg.FrSpin / self._r
        self._RrSpin = Msg.FrSpin * cos(abs(Msg.deltaBi)) / self._r
        self.AckermanCondition()
        self.DifferentialSpinning()
        self.EnforceConstraints()
        # self.CheckAckerman()

        if (self._deltaBi < 0):  # CW: Positive CCW: Negative
            self._pubFrStR.publish(-1*self._delaOut)
            self._pubFrStL.publish(-1*self._delaIn)

            self._pubFrDrR.publish(self._SlowFrSpin)
            self._pubFrDrL.publish(self._FastFrSpin)
            self._pubRrDrR.publish(self._SlowRrSpin)
            self._pubRrDrL.publish(self._FastRrSpin)

        else:
            self._pubFrStL.publish(self._delaOut)
            self._pubFrStR.publish(self._delaIn)
            
            self._pubFrDrR.publish(self._FastFrSpin)
            self._pubFrDrL.publish(self._SlowFrSpin)
            self._pubRrDrR.publish(self._FastRrSpin)
            self._pubRrDrL.publish(self._SlowRrSpin)
            

def cot(c):
    return (1/tan(c))

def main():
	rospy.init_node('Bicycle2AckermanNode')
	pubFrStL = rospy.Publisher('/Pos_Front_Wheel_Left_Steer/command', Float64, queue_size=10)
	pubFrStR = rospy.Publisher('/Pos_Front_Wheel_Right_Steer/command', Float64, queue_size=10)
	pubFrDrR = rospy.Publisher('/Vel_Front_Wheel_Right_Drive/command', Float64, queue_size=10)
	pubFrDrL = rospy.Publisher('/Vel_Front_Wheel_Left_Drive/command', Float64, queue_size=10)
	pubRrDrR = rospy.Publisher('/Vel_Rear_Wheel_Right/command', Float64, queue_size=10)
	pubRrDrL = rospy.Publisher('/Vel_Rear_Wheel_Left/command', Float64, queue_size=10)
	Bolty = Bicycle2Ackerman(pubFrStL,pubFrStR,pubFrDrR,pubFrDrL,pubRrDrR,pubRrDrL)
	rospy.Subscriber('Bicycle_Control', BicycleControl_msg, Bolty.Control_callback)
	rospy.spin()

if __name__ == '__main__':
	main()