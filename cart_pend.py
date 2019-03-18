#####################################
# Simple Pendulum
# Author: Kartik Prakash
# Date: Mar/15/2019
#####################################

from __future__ import division
import matplotlib.pyplot as plt
import scipy.integrate as integrate #used for integration
import numpy as np
from numpy import sqrt, sin,cos


#defining global cart parameters
M = 50 #mass of the cart
m = 5 #mass of the bob
g = 9.81 #gravity
W = np.sqrt(M/5) #cart width made proportional to mass
H = (np.sqrt(M/5))*0.5 #car height made proportional to mass
wr = 0.4 #wheel radius
mr = 0.3*sqrt(m) #mass radius made proportional to mass of the bob
L = 5 #length of the bob

#defining the initial state put degrees
xi = 0 #change here
thi_dash = 150 #change here
xi_dot = 0 #change here
thi_dot_dash = 0 #change here

thi = np.radians(thi_dash)
thi_dot = np.radians(thi_dot_dash)

istate = np.array([xi,xi_dot,thi,thi_dot])

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
    w1 = plt.Circle((w1x,w1y),radius = wr,fc='w')
    w2 = plt.Circle((w2x,w2y),radius = wr,fc='w')
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
    f = 0
    dydx = np.zeros_like(state)
    x,x_dot,th,th_dot = state #unpacking the state
    dydx[0] = x_dot
    dydx[2] = th_dot

    den1 = M + (m*sin(th)*sin(th))
    dydx[1] = (f + (m*g*sin(th)*cos(th)) + m*L*th_dot*th_dot*sin(th))/den1
    den2 = L
    dydx[3] = ((M+m)*g*sin(th) + f*cos(th) + m*L*th_dot*th_dot*sin(th)*cos(th))/den2
    dydx[3] = -dydx[3]

    return dydx


#create a time array for the simulation
dt = 0.05
t = np.arange(0.0,20.0,dt) #samples the interval provided at 0.05

#integrate to get the solution
sol = integrate.odeint(dynamics,istate,t)
x_plot = sol[:,0]
th_plot = sol[:,2]

#plotting after all the things are computed

for i in range(len(x_plot)):
    setup()
    draw_cart(th_plot[i],x_plot[i])
    plt.draw()
    plt.pause(0.000001)
    plt.clf()

plt.show()
