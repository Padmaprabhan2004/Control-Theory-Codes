import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np
import random

M = 0.5
m = 0.2
b = 0.1
I = 0.006
g = 9.8
l = 0.3
k_p,k_d,k_i=300,300,10


dt=0.02
t0=0
t_end=5
t=np.arange(t0,t_end+dt,dt)

x =np.zeros((4, len(t)))
dx =np.zeros_like(x)
y =np.zeros((2, len(t)))
#[[x],[x_dot],[theta],[theta_dot]] is the state of the system

#Initialization
init_x=100
init_dx=0
init_ddx=0
init_theta=np.pi/3
theta_ref=0
#errors
e=np.zeros(len(t))
e_dot=e
e_int=e

#constants in linear time invariant matrices
k1=-b*(I+m*(l**2))/((I*(M+m)+M*m*(l**2)))
k2=(m**2)*g*(l**2)/((I*(M+m)+M*m*(l**2)))
k3=-m*l*b/((I*(M+m)+M*m*(l**2)))
k4=m*g*l*(M+m)/((I*(M+m)+M*m*(l**2)))
k5=(I+m*(l**2))/((I*(M+m)+M*m*(l**2)))
k6=m*l/((I*(M+m)+M*m*(l**2)))
C=np.array([[1,0,0,0],[0,0,1,0]])
D=np.array([[0],[0]])
for i in range(1,len(t)):
    if i==1:
        x[0][0]=init_x
        x[1][0]=init_dx
        x[2][0]=init_theta
        x[3][0]=0
        y[0][0]=init_x
        y[1][0]=init_theta
    e[i-1]=theta_ref-x[2][i-1]

    if i>1:
        e_dot[i-1]=(e[i-1]-e[i-2])/dt
        e_int[i-1]=e_int[i-2]+(e[i-2]+e[i-1])*dt*0.5
    if i==len(t)-1:
        e[-1]=e[-2]
        e_dot[-1]=e[-2]
        e_int[-1]=e[-2]
    U=k_p*e[i-1]+k_d*e_dot[i-1]+k_i*e_int[i-1]
    A=np.array([[0,1,0,0],[0,k1,k2,0],[0,0,0,1],[0,k3,k4,0]])
    B=np.array([[0],[k5],[0],[k6]])
    #print(np.matmul(A,x[:,i-1]).shape)
    dx[:,i]=np.matmul(A,x[:,i-1])+np.reshape(B,(4,))*U
    x[:,i]=x[:,i-1]+(dx[:,i]+dx[:,i-1])*dt*0.5
    y[:,i]=np.matmul(C,x[:,i])+np.reshape(D,(2,))*U

plt.figure(1)
plt.title('X v/s time')
plt.plot(t,y[0,:])
plt.figure(2)
plt.title('Y v/s time')
plt.plot(t,y[1,:])
plt.show()
'''
# Animation Setup
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(-2, 2)
ax.set_ylim(-0.5, 0.5)
ax.set_aspect('equal')
ax.grid()

# Cart and Pendulum Visualization
cart_width = 0.2
cart_height = 0.1
pendulum_length = l

cart = plt.Rectangle((0, 0), cart_width, cart_height, fc="blue", ec="black")
pendulum_line, = ax.plot([], [], lw=2, color="red")
ax.add_patch(cart)

def init():
    cart.set_xy((-cart_width / 2, -cart_height / 2))
    pendulum_line.set_data([], [])
    return cart, pendulum_line

def update(frame):
    # Update cart position
    cart_center_x = x[0, frame]
    cart.set_xy((cart_center_x - cart_width / 2, -cart_height / 2))

    # Update pendulum position
    theta = x[2, frame]
    pendulum_x = [cart_center_x, cart_center_x + pendulum_length * np.sin(theta)]
    pendulum_y = [0, -pendulum_length * np.cos(theta)]
    pendulum_line.set_data(pendulum_x, pendulum_y)

    return cart, pendulum_line
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, frames=len(t), init_func=init, blit=True, interval=dt * 1000)
plt.title("Inverted Pendulum on a Cart")
plt.show()

'''
