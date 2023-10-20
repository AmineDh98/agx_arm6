# Import necessary libraries
from lab2_robotics import * # Import our library (includes Numpy)
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition (planar 2 link manipulator)
d = np.zeros(2)           # displacement along Z-axis
q = np.array([0.2, 0.5])  # rotation around Z-axis (theta)
a = np.array([0.75, 0.5]) # displacement along X-axis
alpha = np.zeros(2)       # rotation around X-axis 

# Simulation params
dt = 0.01 # Sampling time
Tt = 10 # Total simulation time
tt = np.arange(0, Tt, dt) # Simulation time vector

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Kinematics')
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'r-', lw=1) # End-effector path

fig1 = plt.figure()
ax1 = fig1.add_subplot(111, autoscale_on=False, xlim=(0, 10), ylim=(0, 16))
ax1.set_title('Joint position')
ax1.set_xlabel('time[s]')
ax1.set_ylabel('angle[rad]')
ax1.set_aspect('auto')
ax1.grid()
# Previous commands are defining the names and units of the axes, as well as the name of the figure and type of the grid in the background

t = 0
# Starting value of time
q1 = q[0]
# Starting value of angle q1
q2 = q[1]
# Starting value of angle q2

xx = [t]
# Creating the array for the time axis
yy1 = [q1] 
# Creating the array for y axis for q1
yy2 = [q2]
# Creating the array for y axis for q1

for i in range(1000):
# Since the time interval should be 10s and dt = 0.01s, the loop for recording the angles during the time should be repeated 1000 times

    t += dt
    # Increment of the time
    q1 = q1 + dt * 0.6
    # Increment of the angle q1 where 0.6 is the dq1 value
    q2 = q2 + dt * 1.5
    # Increment of the angle q2 where 1.5 is the dq2 value

    xx.append(t)
    # Adding values of time to the array
    yy1.append(q1)
    # Adding values of q1 to the array
    yy2.append(q2)
    # Adding values of q2 to the array

# Memory
PPx = []
PPy = []

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    return line, path

# Simulation loop
def simulate(t):
    global d, q, a, alpha
    global PPx, PPy
    
    # Update robot
    T = kinematics(d, q, a, alpha)
    dq = np.array([0.6, 1.5]) # Define how joint velocity changes with time!
    q = q + dt * dq
    
    # Update drawing
    PP = robotPoints2D(T)
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    
    return line, path
plt.plot(xx, yy1, label = 'q1')
plt.plot(xx,yy2, label = 'q2')
plt.legend()
# Noting the x and y values during the time and showing the respective legend in the upper left corner of the figure window

# Run simulation
animation = anim.FuncAnimation(fig, simulate, tt, 
                                interval=10, blit=True, init_func=init, repeat=False) #ako stavimo True ponavlja se beskonacno
plt.show()
