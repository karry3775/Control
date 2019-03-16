#####################################
# Simple Pendulum
# Author: Kartik Prakash
# Date: Mar/15/2019
#####################################
from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation


#define constant parameters
g = 9.81
l = 1.0
m = 1.0
b = 0.5

# print("Dynamics with non-linearity, damped, without control")
print("Running feedback_linearizer")
thd = float(raw_input("Enter the desired theta"))

def dynamics_small_undamped(state,t):
    """
    This function returns the derivative of state vector
    under the assumption that the angles are small without damping and hence the
    dynamics is linear
    """
    #print("Dynamics with linearity, undamped, without control")
    dydx = np.zeros_like(state) #will define the system dynamics
    dydx[0] = state[1] #represents theta_dot
    dydx[1] = -(g/l)*state[0] #represents theta_double_dot

    return dydx

def dynamics_small_damped(state,t):
    """
    This function returns the derivative of state vector
    with damping
    """
    #print("Dynamics with non-linearity, damped, without control")

    dydx = np.zeros_like(state) #will define the system dynamics
    dydx[0] = state[1] #represents theta_dot
    k = b/(m*l*l)
    dydx[1] =-(g/l)*sin(state[0]) - (k*state[1]) #represents theta_double_dot

    return dydx

def feedback_linearizer(state,t):
    """
    This function returns the dynamics as influenced by feedback_linearizer
    """
    dydx = np.zeros_like(state)
    dydx[0] = state[1]
    k = b/(m*l*l)

    dydx[1] = (g/l)*sin(state[0]) - (k*state[1])
    return dydx

def constant_torque(state,t):
    """
    This function returns the dynamics as influenced by constant_torque
    """
    dydx = np.zeros_like(state)
    dydx[0] = state[1]
    k = b/(m*l*l)

    dydx[1] = thd/10 -(g/l)*sin(state[0]) - (k*state[1])
    return dydx

#create a time array for plotting
dt = 0.05
t = np.arange(0.0,20.0,dt)

#define initial values for angle(th) and angular velocity(w)
th = 120#degrees
w = 0.0 #degrees per second

#initial state
state = np.radians([th,w])

#integrate the ode using scipy.integrate
sol = integrate.odeint(constant_torque,state,t) #change constant_torque to other methods to check their functionality

#defining x and y positions
x = l*sin(sol[:,0])
y = -l*cos(sol[:,0])

#figure adjustments
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text

def animate(i):
    thisx = [0, x[i]]
    thisy = [0, y[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    return line, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(y)),
                              interval=25, blit=True, init_func=init)
plt.show()
