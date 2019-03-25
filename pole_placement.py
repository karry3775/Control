#!/usr/bin/env python3

#####################################
# CONTROL OF A CART_POLE
# AUTHOR: KARTIK PRAKASH
# DATE: Mar/24/2019
#####################################

"""
IMPORTS
"""
from __future__ import division
import matplotlib.pyplot as plt
import scipy.integrate as integrate #used for integration
import numpy as np
from numpy import sqrt, sin,cos
import control
from numpy.linalg import matrix_rank as rank

"""
SYSTEM SPECIFICATION
"""

#defining global cart parameters
M = 50 #mass of the cart
m = 5 #mass of the bob
g = 9.81 #gravity
W = np.sqrt(M/5) #cart width made proportional to mass
H = (np.sqrt(M/5))*0.5 #car height made proportional to mass
wr = 0.4 #wheel radius
mr = 0.3*sqrt(m) #mass radius made proportional to mass of the bob
L = 5 #length of the bob
b = 0 #damping coefficient

#defining the initial state put degrees
xi = 0 #change here
thi_dash = 180 #change here
xi_dot = 0 #change here
thi_dot_dash = 0 #change here

# thi = np.radians(thi_dash)
thi = np.pi
thi_dot = np.radians(thi_dot_dash)

istate = np.array([xi,xi_dot,thi,thi_dot])

"""
FUNCTION DEFINITION
"""
def setup():
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.gca().set_aspect('equal',adjustable='box')
    #plotting the line
    line = plt.plot([-10,10],[0,0],'-',linewidth=3.0,color='k')

def draw_cart(th,x):
    # thd = 60
    # th = thd*((np.pi)/180)
    global M,m,W,H,wr,mr
    #plotting the base
    left_corner = ((x-W/2),(2*wr)) #tuple of x and y coords of left_corner
    rect = plt.Rectangle(left_corner,W,H,fc='r')
    plt.gca().add_patch(rect)
    #plotting the wheels
    w1x = x - ((0.5*W)/2)
    w1y = wr
    w2x = x + ((0.5*W)/2)
    w2y = wr
    w1 = plt.Circle((w1x,w1y),radius = wr,fc='y')
    w2 = plt.Circle((w2x,w2y),radius = wr,fc='y')
    plt.gca().add_patch(w1)
    plt.gca().add_patch(w2)
    draw_pend(th,x)

def draw_pend(th,x):
    global L,H,wr,mr
    xp = x + L*sin(th)
    y_cart = H/2 + 2*wr
    yp = y_cart - L*cos(th)
    bob = plt.Circle((xp,yp),radius=mr,fc='g')
    plt.gca().add_patch(bob)
    plt.plot([x,xp],[y_cart,yp],color='k',linewidth=2.0)

def dynamics(state,t):
    """
    Takes the initial state and time as an input and returns the dynamics
    """
    global M,m
    f = control_upright(state)
    # f = 0
    dydx = np.zeros_like(state)
    x,x_dot,th,th_dot = state #unpacking the state
    dydx[0] = x_dot
    dydx[2] = th_dot

    den1 = M + (m*sin(th)*sin(th))
    dydx[1] = (f + (m*g*sin(th)*cos(th)) + m*L*th_dot*th_dot*sin(th) + (b/L)*(th_dot*cos(th)))/den1
    den2 = L*den1
    dydx[3] = (((M+m)*g*sin(th) + f*cos(th) + m*L*th_dot*th_dot*sin(th)*cos(th))/den2) + (b/(m*L*L))*th_dot
    dydx[3] = -dydx[3]

    return dydx


def sim_cartpend(istate):
    #create a time array for the simulation
    dt = 0.5
    t = np.arange(0.0,100.0,dt) #samples the interval provided at 0.05

    #integrate to get the solution
    sol = integrate.odeint(dynamics,istate,t)
    x_plot = sol[:,0]
    th_plot = sol[:,2]
    # plt.plot(x_plot)
    # plt.plot(th_plot)
    #plotting after all the things are computed
    for j in range(3):
        print("iteration: " + str(j))
        for i in range(len(x_plot)):
            plt.clf()
            setup()
            draw_cart(th_plot[i],x_plot[i])
            plt.draw()
            plt.pause(0.00001)

    plt.show()


def control_upright(state):
    global b, m , L ,M ,g
    x = np.reshape(state,(4,1))
    x_ref = np.array([[1],[0],[np.pi],[0]])
    x = x - x_ref
    a1 = (m/M)*g
    a2 = ((m+M)*g)/(M*L)
    a3 = -b/(m*L*L)
    A = np.array([[0,1,0,0],[0,0,a1,0],[0,0,0,1],[0,0,a2,a3]])
    b1 = 1/M
    b2 = 1/(M*L)
    B = np.array([[0],[b1],[0],[b2]])
    u = 0
    print("Is it controllable?")
    C_curl = control.ctrb(A,B) #returns the controllability matrix
    r = rank(C_curl)
    if (r == 4):
        print("The system is controllable")
        #as the system is controllable we can arbitrarily place the eigen values/poles
        # p = [-1,-0.1,-0.2,-0.5]
        p = [-1,-1,-1,-1]
        K = control.acker(A,B,p)
        print("The K matrix is: ")
        print(K)
        print(K.shape)
        #now lets return the control law
        u = -np.dot(K,x)
        print("the control law is: ")
        print(u)
    else:
        print("The system can't be controlled")

    return u

"""
USAGE

from pole_placement import sim_cartpend #pole_placement should be replaced by the name you are saving this file
import numpy as np
th = 178
thr = np.radians(th)
th_dot = 2
x_dot = 2
state = np.array([0,x_dot,thr,th_dot])

sim_cartpend(state)
"""

