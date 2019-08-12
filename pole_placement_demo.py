from pole_placement import sim_cartpend #pole_placement should be replaced by the name you are saving this file
import numpy as np

th = 150
thr = np.radians(th)
th_dot = 2
x_dot = 2
state = np.array([0,x_dot,thr,th_dot])

sim_cartpend(state)
