#!/usr/bin/env python       
# source activate ros_env
# 

import time
import rospy
import sys, os
import numpy as np
from MPC_Sim.msg import BicycleControl_msg
from MPC_Sim.msg import Goal_Ackerman
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32, String, Float32

sys.path.append(os.path.join(sys.path[0],'..','scripts','casadi-linux-py27-v3.4.4-64bit'))
from casadi import *

class AckermanMPC(object):
    def __init__(self,BicyclePub):
        self._isRecorded = 0 
        self._BicyclePub = BicyclePub
        self._X_State_Limit = 50
        self._Y_State_Limit = 50
        # raw_input("Press Enter to continue...")
        self._X0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Initial Position
        self._Xs = [20.0, 5.0,0.0, 0.0, 0.0, 0.0]    # Reference posture.
        self.Initialize_MPC()
        self._GInd = 0

    def Initialize_MPC(self):
        self._T = 0.2 # Sampling time [s]
        self._N = 20   # Prediction horizon
        self._r = 0.2  # Wheel Radius
        self._L = 1.3  # Wheel Base/2
        #   Bolt Maximum Speed = 145 km/h ()
        #   Sp : Front wheel spinning speed (rad/s)
        #   St : Front wheel steerng angle (rad)
        self._Sp_max = 200.0
        self._Sp_min = -self._Sp_max/7         # Bolt Spinning Speed Constraints
        self._St_max = pi/6 
        self._St_min = -self._St_max           # Bolt Steering Angle Constraints
        self._dSp_max = 2               # Rad/s**2
        self._dSp_min = self._dSp_max * -2
        self._dSt_max = 0.1             # rad/s
        self._dSt_min = -self._dSt_max
        # States
        self._x = SX.sym('x')
        self._y = SX.sym('y')
        self._v = SX.sym('v')
        self._Spin = SX.sym('Spin')
        self._Steer = SX.sym('Steer')
        self._theta  = SX.sym('theta')
        self._states  = vertcat(self._x,self._y,self._theta,self._v ,self._Spin ,self._Steer)
        self._n_states  = SX.size(self._states)
        self._StAct = 0
        self._SpAct = 0

        # Sytem Model
        self._Sp = SX.sym('Sp')   # System Inputs
        self._St = SX.sym('St')   # System Inputs
        self._controls = vertcat(self._Sp,self._St)    # System Control Inputs
        self._n_control = SX.size(self._controls)

        self._Vx = self._r * self._Sp * cos(self._St)
        self._Vy = self._r * self._Sp * sin(self._St)/2
        self._W = self._r * self._Sp * sin(self._St)/(2*self._L)

        self._rhs = vertcat( self._Vx * cos(self._theta) - self._Vy * sin(self._theta),
        self._Vx * sin(self._theta) + self._Vy * cos(self._theta),
        self._W,
        self._r * self._Sp,
        self._Sp,
        self._St)

        self._f = Function('f',[self._states,self._controls],[self._rhs]) # Nonlinear mapping Function f(x,u)
        
        self._U = SX.sym('U',self._n_control[0],self._N)                  # Decision variables (controls)
        self._P = SX.sym('P',self._n_states[0] + self._n_states[0] )      # Parameters which include the initial and the reference state of the robot

        self._X = SX.sym('X',self._n_states[0],(self._N+1))               # A matrix that represents the states over the optimization problem

        self._X[:,0] = self._P[0:6]
        
        # Compute solution symbolically
        for n in range(self._N):
            self._ste = self._X[:,n]
            self._con = self._U[:,n]
            self._f_value = self._f(self._ste,self._con)
            self._st_next =self._ste[0:3] + (self._T*self._f_value[0:3])
            self._X[0:3,n+1] = self._st_next
            self._X[3,n+1] = self._f_value[3] 
            self._X[4:6,n+1] = self._f_value[4:6]

        self._obj = 0 # Objective function
        # self._obj = SX([]) # Objective function
        self._g = SX([]) # Constraints vector

        # Weighing matrices (States)
        # self._Q = DM([[50,0,0],[0,150,0],[0,0,100]])
        self._Q = DM([[10,0,0],[0,30,0],[0,0,10]])

        # Weighing matrices (Controls)
        self._R = DM([[1,0],[0,0.1]])

        self._con_old = DM.zeros(self._n_control[0],1)

        # Compute Objective
        for k in range(self._N):
            self._st = self._X[0:3,k]
            self._con = self._U[:,k]
            objT1 = mtimes((self._st[0:3]-self._P[6:9]).T, mtimes(self._Q ,(self._st[0:3]-self._P[6:9])))
            objT2 = mtimes(self._con.T-self._con_old.T,mtimes(self._R, self._con-self._con_old))
            # self._obj = self._obj + mtimes((self._st-self._P[3:6]).T, mtimes(self._Q ,(self._st-self._P[3:6]))) + mtimes(self._con.T-self._con_old.T,mtimes(self._R, self._con-self._con_old))
            self._obj = self._obj + objT1 + objT2
            self._con_old = self._con

        # Compute Constraints
        for k in range(self._N+1):
            self._g = vertcat(self._g,self._X[0,k])
            self._g = vertcat(self._g,self._X[1,k])
        
        for k in range(self._N-1):
            dU = self._U[:,k] - self._X[4:6,k]
            self._g = vertcat(self._g, dU)
       
        # Make the decision variables on column vector
        self._OPT_variables = reshape(self._U, 2*self._N, 1)

        self._nlp_prob = {'f':self._obj, 'x':self._OPT_variables, 'g':self._g,'p':self._P}

        # Pick an NLP solver
        self._MySolver = 'ipopt'

        # Solver options
        self._opts = {'ipopt.max_iter':100,
                'ipopt.print_level':0,
                'print_time':0,
                'ipopt.acceptable_tol':1e-8,
                'ipopt.acceptable_obj_change_tol':1e-6
                }

        self._solver = nlpsol('solver',self._MySolver,self._nlp_prob,self._opts)

        # inequality constraints (state constraints)

        self._lbx = DM.zeros(2*self._N,1)
        self._ubx = DM.zeros(2*self._N,1)

        self._lbx[0:2*self._N-1:2,0] = self._Sp_min
        self._lbx[1:2*self._N:2,0] = self._St_min
        self._ubx[0:2*self._N-1:2,0] = self._Sp_max
        self._ubx[1:2*self._N:2,0] = self._St_max

        self._lbg = DM.zeros(4*self._N,1)
        self._ubg = DM.zeros(4*self._N,1)

        self._lbg[0:self._N*2+2,0] = DM(-self._X_State_Limit)
        self._ubg[0:self._N*2+2,0] = DM(self._X_State_Limit)

        self._lbg[self._N*2+2:2*self._N*2-2+1:2,0] = DM(self._dSp_min)
        self._lbg[self._N*2+2+1:2*self._N*2:2,0] = DM(self._dSt_min)

        self._ubg[self._N*2+2:2*self._N*2-2+1:2,0] = DM(self._dSp_max)
        self._ubg[self._N*2+2+1:2*self._N*2:2,0] = DM(self._dSt_max)

        self._args = {}
        self._args['lbg'] = self._lbg       # dU and States constraints
        self._args['ubg'] = self._ubg       # dU and States constraints
        self._args['lbx'] = self._lbx       #  input constraints
        self._args['ubx'] = self._ubx       #  input constraints
        self._u0  = DM.zeros(self._N,2) # Two control inputs
        self._T_old = 0
        
    def Gaol_callBack(self,MsgG):
        self._Xs = [MsgG.x, MsgG.y, MsgG.theta,0,0,0]
    
    def cot(self,x):
        return (1/tan(x))

    def JointState_callBack(self,Joints):
        self._SpAct = (Joints.velocity[0] + Joints.velocity[2])/2
        if (Joints.position[1] < 0.0005 and Joints.position[1] > -0.0005):
            self._StAct = 00
        else:
            self._StAct = -atan(2/( self.cot(Joints.position[1]) + self.cot(Joints.position[3])))


    def States_callBack(self,Msg):
        seconds = rospy.get_time()

        if((seconds - self._T_old ) > self._T):
            self._T_old = seconds
            
            self._xDis = Msg.pose[1].position.x
            self._yDis = Msg.pose[1].position.y
            self._Vact = sqrt(Msg.twist[1].linear.x**2 + Msg.twist[1].linear.y**2)
            orientation_list = [Msg.pose[1].orientation.x,Msg.pose[1].orientation.y,Msg.pose[1].orientation.z,Msg.pose[1].orientation.w]
            (roll, pitch, self._yaw) = euler_from_quaternion (orientation_list)
            
            self._Error = np.linalg.norm([self._xDis-self._Xs[0], self._yDis-self._Xs[1],self._yaw-self._Xs[2]])
            print '---------------------------'
            print 'Err X: {}'.format(self._xDis-self._Xs[0])
            print 'Err Y: {}'.format(self._yDis-self._Xs[1])
            print 'Err Yaw: {}'.format(self._yaw-self._Xs[2])

            if(self._Error > 3.5e-1):
                self._T_old_Running = seconds
                print 'Error: {}'.format(self._Error)
                R = [self._xDis, self._yDis,self._yaw, self._Vact,self._SpAct ,self._StAct ,self._Xs[0],self._Xs[1],self._Xs[2], self._Xs[3], self._Xs[4], self._Xs[5]]
                self._args['p'] = R

                self._args['x0'] = reshape(self._u0.T, 2*self._N, 1)    # initial condition for optimization variable   

                self.sol = self._solver(
                x0 = self._args['x0'],
                lbx = self._args['lbx'],
                ubx = self._args['ubx'],
                lbg = self._args['lbg'],
                ubg = self._args['ubg'],
                p = self._args['p'])
                
                self._u = reshape(self.sol['x'].T,2,self._N).T
                self._u0[0:self._N-1,:] = self._u[1:self._N,:]
                self._u0[self._N-1,:] = self._u[self._N-1,:]
                BicycleMotion = [self._u[0,0].__float__(),self._u[0,1].__float__()]
                self.BiMSG = BicycleControl_msg()
                self.BiMSG.FrSpin = BicycleMotion[0]
                self.BiMSG.deltaBi = BicycleMotion[1]  
                self._BicyclePub.publish(self.BiMSG)

            else:
                print 'ARRIVED'
                print self._GInd
                self.BiMSG = BicycleControl_msg()
                self.BiMSG.FrSpin = 0
                self.BiMSG.deltaBi = 0
                self._BicyclePub.publish(self.BiMSG)


def main():
    rospy.init_node('Ackerman_Point_Stabilization')

    BicyclePub = rospy.Publisher('Bicycle_Control', BicycleControl_msg, queue_size=10)
    Bolt = AckermanMPC(BicyclePub)

    rospy.Subscriber('setGoal', Goal_Ackerman, Bolt.Gaol_callBack)
    rospy.Subscriber('/joint_states', JointState, Bolt.JointState_callBack)
    rospy.Subscriber('/gazebo/model_states', ModelStates, Bolt.States_callBack)
    rospy.spin()

if __name__ == '__main__':
    main()
