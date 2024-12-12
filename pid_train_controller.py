import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np
import random


trials=5
inc_angle=np.pi/4
g=10
mass_cart=100

#constants
k_p,k_d,k_i=300,300,10

trials_global=trials


def sample_cube_xy(inc_angle):
    x=random.uniform(0,120)
    y=random.uniform(120*np.tan(inc_angle)+20+6.5,120*np.tan(inc_angle)+40+6.5)
    return x,y

dt=0.02
t0=0
t_end=5
t=np.arange(t0,t_end+dt,dt)
f_g=mass_cart*g


#init
disp_rail=np.zeros((trials,len(t)))
v_rail=np.zeros((trials,len(t)))
a_rail=np.zeros((trials,len(t)))
x_train=np.zeros((trials,len(t)))
y_train=np.zeros((trials,len(t)))
e=np.zeros((trials,len(t)))
e_dot=np.zeros((trials,len(t)))
e_int=np.zeros((trials,len(t)))


x__cube=np.zeros((trials,len(t)))
y_cube=np.zeros((trials,len(t)))

F_ga=f_g*np.sin(inc_angle)
init_pos_x=120
init_pos_y=120*np.tan(inc_angle)+6.5
init_disp_rail=(init_pos_x**2+init_pos_y**2)**0.5
init_vel_rail=0
init_a_rail=0


#global pos. used for determining the dimensions of animation
init_pos_x_global=init_pos_x
trials_magn=trials
history=np.ones(trials)
while(trials>0):
    pos_x_cube_ref,pos_y_cube_ref=sample_cube_xy(inc_angle)
    time=trials_magn-trials
    #these lines are correct due to numpy broadcasting.
    x__cube[time]=pos_x_cube_ref
    y_cube[time]=pos_y_cube_ref-0.5*g*(t**2)
    win=False
    delta=1
    
    for i in range(1,len(t)):
        if i==1:
            disp_rail[time][0]=init_disp_rail
            x_train[time][0]=init_pos_x
            y_train[time][0]=init_pos_y
            v_rail[time][0]=init_vel_rail
            a_rail[time][0]=init_a_rail
        #horizontal error
        e[time][i-1]=pos_x_cube_ref-x_train[time][i-1]


        if i>1:
            e_dot[time][i-1]=(e[time][i-1]-e[time][i-2])/dt
            e_int[time][i-1]=e_int[time][i-2]+(e[time][i-2]+e[time][i-1])*0.5*dt
        
        if i==len(t)-1:
            e[time][-1]=e[time][-2]
            e_dot[time][-1]=e_dot[time][-2]
            e_int[time][-1]=e_int[time][-2]


        F_app=k_p*e[time][i-1]+k_d*e_dot[time][i-1]+k_i*e_int[time][i-1]
        F_net=F_app+F_ga
        a_rail[time][i]=F_net/mass_cart
        v_rail[time][i]=v_rail[time][i-1]+(a_rail[time][i]+a_rail[time][i-1])*0.5*dt
        disp_rail[time][i]=disp_rail[time][i-1]+(v_rail[time][i]+v_rail[time][i-1])*0.5*dt
        x_train[time][i]=disp_rail[time][i]*np.cos(inc_angle)
        y_train[time][i]=disp_rail[time][i]*np.sin(inc_angle)+6.5
        

        #check two conditions, one for horizontal dist and other vertical dist
        if (x_train[time][i]-5<x__cube[time][i]+3 and x_train[time][i]+5>x__cube[time][1]-3) or win==True:
            if(y_train[time][i]+3<y_cube[time][i]-2 and y_train[time][i]+8>y_cube[time][i]+2) or win==True:
                win=True
                if delta==1:
                    change=x_train[time][i]-x__cube[time][i]
                    delta=0
                x__cube[time][i]=x_train[time][i]-change
                y_cube[time][i]=y_train[time][i]+5
    
    init_disp_rail=disp_rail[time][-1]
    init_pos_x=x_train[time][-1]+v_rail[time][-1]*np.cos(inc_angle)*dt
    init_pos_y=y_train[time][-1]+v_rail[time][-1]*np.sin(inc_angle)*dt
    init_vel_rail=v_rail[time][-1]
    init_a_rail=a_rail[time][-1]
    history[time]=delta
    trials=trials-1


import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np

# Assuming your initialization and simulation code above this section

len_t = len(t)
frame_amount = len(t) * trials_global

# Define the update function for the animation
def update_plot(num):
    index = int(num / len_t)
    frame = num - index * len_t

    # Update platform position
    platform.set_data(
        [x_train[index][frame] - 3.1, x_train[index][frame] + 3.1],
        [y_train[index][frame], y_train[index][frame]]
    )

    # Update cube position
    cube.set_data(
        [x__cube[index][frame] - 1, x__cube[index][frame] + 1],
        [y_cube[index][frame], y_cube[index][frame]]
    )



    # Update graphs
    displ_rail_f.set_data(t[:frame], disp_rail[index][:frame])
    v_rail_f.set_data(t[:frame], v_rail[index][:frame])
    a_rail_f.set_data(t[:frame], a_rail[index][:frame])
    e_f.set_data(t[:frame], e[index][:frame])
    e_dot_f.set_data(t[:frame], e_dot[index][:frame])
    e_int_f.set_data(t[:frame], e_int[index][:frame])

    return displ_rail_f, v_rail_f, a_rail_f, e_f, e_dot_f, e_int_f, platform, cube, success, again

# Setup figure and axes
fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=(0.8, 0.8, 0.8))
gs = gridspec.GridSpec(4, 3)

# Main game window
ax_main = fig.add_subplot(gs[0:3, 0:2], facecolor=(0.9, 0.9, 0.9))
ax_main.set_xlim(0, init_pos_x_global)
ax_main.set_ylim(0, init_pos_x_global)
ax_main.set_xticks(np.arange(0, init_pos_x_global + 1, 10))
ax_main.set_yticks(np.arange(0, init_pos_x_global + 1, 10))
ax_main.grid(True)

# Annotations
copyright = ax_main.text(0, 122, 'Plots', size=12)
rail = ax_main.plot([0, init_pos_x_global], [5, init_pos_x_global * np.tan(inc_angle) + 5], 'k', linewidth=6)
platform, = ax_main.plot([], [], 'b', linewidth=18)
cube, = ax_main.plot([], [], 'k', linewidth=14)

# Text boxes for success or retry messages
bbox_props_success = dict(boxstyle='square', fc=(0.9, 0.9, 0.9), ec='g', lw=1.0)
success = ax_main.text(40, 60, '', size='20', color='g', bbox=bbox_props_success)

bbox_props_again = dict(boxstyle='square', fc=(0.9, 0.9, 0.9), ec='r', lw=1.0)
again = ax_main.text(30, 60, '', size='20', color='r', bbox=bbox_props_again)

# Plot windows for displacement, velocity, and acceleration
ax1v = fig.add_subplot(gs[0, 2], facecolor=(0.9, 0.9, 0.9))
displ_rail_f, = ax1v.plot([], [], '-b', linewidth=2, label='displ. on rails [m]')
ax1v.set_xlim(t0, t_end)
ax1v.set_ylim(np.min(disp_rail) * 0.9, np.max(disp_rail) * 1.1)
ax1v.grid(True)
ax1v.legend(loc='lower left', fontsize='small')

ax2v = fig.add_subplot(gs[1, 2], facecolor=(0.9, 0.9, 0.9))
v_rail_f, = ax2v.plot([], [], '-b', linewidth=2, label='velocity on rails [m/s]')
ax2v.set_xlim(t0, t_end)
ax2v.set_ylim(np.min(v_rail) * 0.9, np.max(v_rail) * 1.1)
ax2v.grid(True)
ax2v.legend(loc='lower left', fontsize='small')

ax3v = fig.add_subplot(gs[2, 2], facecolor=(0.9, 0.9, 0.9))
a_rail_f, = ax3v.plot([], [], '-b', linewidth=2, label='accel. on rails [m/s^2]')
ax3v.set_xlim(t0, t_end)
ax3v.set_ylim(np.min(a_rail) * 0.9, np.max(a_rail) * 1.1)
ax3v.grid(True)
ax3v.legend(loc='lower left', fontsize='small')

# Plot windows for error metrics
ax1h = fig.add_subplot(gs[3, 0], facecolor=(0.9, 0.9, 0.9))
e_f, = ax1h.plot([], [], '-b', linewidth=2, label='horizontal error [m]')
ax1h.set_xlim(t0, t_end)
ax1h.set_ylim(np.min(e) * 0.9, np.max(e) * 1.1)
ax1h.grid(True)
ax1h.legend(loc='lower left', fontsize='small')

ax2h = fig.add_subplot(gs[3, 1], facecolor=(0.9, 0.9, 0.9))
e_dot_f, = ax2h.plot([], [], '-b', linewidth=2, label='change of horiz. error [m/s]')
ax2h.set_xlim(t0, t_end)
ax2h.set_ylim(np.min(e_dot) * 0.9, np.max(e_dot) * 1.1)
ax2h.grid(True)
ax2h.legend(loc='lower left', fontsize='small')

ax3h = fig.add_subplot(gs[3, 2], facecolor=(0.9, 0.9, 0.9))
e_int_f, = ax3h.plot([], [], '-b', linewidth=2, label='sum of horiz. error [m*s]')
ax3h.set_xlim(t0, t_end)
ax3h.set_ylim(np.min(e_int) * 0.9, np.max(e_int) * 1.1)
ax3h.grid(True)
ax3h.legend(loc='lower left', fontsize='small')

# Create animation
pid_ani = animation.FuncAnimation(fig, update_plot, frames=frame_amount, interval=10, repeat=False, blit=True)
plt.show()
