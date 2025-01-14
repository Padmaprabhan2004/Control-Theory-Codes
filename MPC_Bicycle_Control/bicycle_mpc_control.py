import numpy as np
import matplotlib.pyplot as plt
import bicycle_control as bc
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from qpsolvers import *
np.set_printoptions(suppress=True)

support=bc.SupportFunctionsBicycle()
constants=support.constants

Ts=constants['Ts']
outputs=constants['outputs'] # [psi, Y]
hz = constants['hz'] # horizon prediction period
time_length=constants['time_length'] # duration of the manoeuvre
inputs=constants['inputs']
x_lim=constants['x_lim']
y_lim=constants['y_lim']
trajectory=constants['trajectory']

#creation of time array for trajectory generator
t=np.zeros((int(time_length/Ts+1)))
for i in range(1,len(t)):
    t[i]=np.round(t[i-1]+Ts,2)

x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref=support.trajectory_generator(t)
simulation_length=len(t)#no.of iterations
ref_signals=np.zeros(len(X_ref)*outputs)

k=0
for i in range(0,len(ref_signals),outputs):
    ref_signals[i]=x_dot_ref[k]
    ref_signals[i+1]=psi_ref[k]
    ref_signals[i+2]=X_ref[k]
    ref_signals[i+3]=Y_ref[k]
    k+=1

#ref_signals=[x_dof_ref,psi_ref,X_ref,Y_ref]


##----------------INITIALIZATION------------------##
x_dot=x_dot_ref[0]
y_dot=y_dot_ref[0]
psi=psi_ref[0]
psi_dot=0
X=X_ref[0]
Y=Y_ref[0]

states=np.array([x_dot,y_dot,psi,psi_dot,X,Y])
states_total=np.zeros((simulation_length,len(states))) #for plotting
states_total[0][:]=states

##-----------------ACCELERATIONS-----------------##
x_dot_dot=0
y_dot_dot=0
psi_dot_dot=0

accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
accelerations_total=np.zeros((simulation_length,len(accelerations))) #for plotting

##---------X_OPT AND Y_OPT FOR PLOTTING------------##
X_opt_total=np.zeros((simulation_length,hz))
Y_opt_total=np.zeros((simulation_length,hz))


##----------------------INPUTS---------------------##
U1=0
U2=0
UTotal=np.zeros((simulation_length,inputs))
UTotal[0][0]=U1
UTotal[0][1]=U2


##-------------INIT THE MPC LOOP-------------------##
k=0
du=np.zeros((inputs*hz,1)) #[del_del0,del_a0,del_del1,del_a1,........del_del20,del_a20]


t_ani=[]
x_dot_ani=[]
psi_ani=[]
X_ani=[]
Y_ani=[]
delta_ani=[]

for i in range(0,simulation_length-1):
    Ad,Bd,Cd,Dd=support.state_space(states,U1,U2)

    #converting delta and acc to delta_delta and delta_acc-->
    x_aug_t=np.transpose([np.concatenate((states,[U1,U2]),axis=0)])

    k=k+outputs
    if (k+outputs*hz)<=len(ref_signals):
        r=ref_signals[k:k+outputs*hz]
    else:
        r=ref_signals[k:]
        hz=hz-1 #as we move to end

    ##------------MPC CONTROLLER-------------##
    Hdb,Fdbt,Cdb,Adc,G,ht=support.mpc_simplification(Ad,Bd,Cd,Dd,hz,x_aug_t,du)
    ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
    #print(ft.shape,Hdb.shape)

    ##-----------------QPSOLVER-----------------##
    try:
        #print(G.shape,ht.shape)
        #print(Hdb)
        #print(ft)
        #print(G,ht)
        du=solve_qp(Hdb,ft,G,ht,solver="cvxopt")
        du=np.transpose([du])
    except ValueError as ve:
        print('Error occured.')
        print('Solution is indeterminate. QPSolvers failed to converge!')
        print('Iteration : {}'.format(i))
        break
    #print(du.shape)
    if (i%200==0):
        print("MPC Computation progress : {}%".format(i*100/simulation_length))
    U1=U1+du[0][0]
    U2=U2+du[1][0]
    UTotal[i+1][0]=U1
    UTotal[i+1][1]=U2


    states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states(states,U1,U2)
    states_total[i+1][0:len(states)]=states

    accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
    accelerations_total[i+1][:]=accelerations

    if i%5==1:
        t_ani=np.concatenate([t_ani,[t[i]]])
        x_dot_ani=np.concatenate([x_dot_ani,[states[0]]])
        psi_ani=np.concatenate([psi_ani,[states[2]]])
        X_ani=np.concatenate([X_ani,[states[4]]])
        Y_ani=np.concatenate([Y_ani,[states[5]]])
        delta_ani=np.concatenate([delta_ani,[U1]])




##-----------------------------ANIMATION--------------------------------##
def update_plot(num):
    hz=constants['hz']
    car_1.set_data([X_ani[num]-lr*np.cos(psi_ani[num]),X_ani[num]+lf*np.cos(psi_ani[num])],
                   [Y_ani[num]-lr*np.sin(psi_ani[num]),Y_ani[num]+lf*np.sin(psi_ani[num])])
    car_determined.set_data(X_ani[:num],Y_ani[:num])
    x_dot.set_data(t_ani[:num],x_dot_ani[:num])
    yaw_angle.set_data(t_ani[:num],psi_ani[:num])
    X_position.set_data(t_ani[:num],X_ani[:num])
    Y_position.set_data(t_ani[:num],Y_ani[:num])

    car_1_body.set_data([-lr*np.cos(psi_ani[num]),lf*np.cos(psi_ani[num])],
                        [-lr*np.sin(psi_ani[num]),lf*np.sin(psi_ani[num])])

    car_1_body_extension.set_data([0,(lf+40)*np.cos(psi_ani[num])],
        [0,(lf+40)*np.sin(psi_ani[num])])
    
    car_1_front_wheel.set_data([lf*np.cos(psi_ani[num])-0.5*np.cos(psi_ani[num]+delta_ani[num]),lf*np.cos(psi_ani[num])+0.5*np.cos(psi_ani[num]+delta_ani[num])],
        [lf*np.sin(psi_ani[num])-0.5*np.sin(psi_ani[num]+delta_ani[num]),lf*np.sin(psi_ani[num])+0.5*np.sin(psi_ani[num]+delta_ani[num])])

    car_1_front_wheel_extension.set_data([lf*np.cos(psi_ani[num]),lf*np.cos(psi_ani[num])+(0.5+40)*np.cos(psi_ani[num]+delta_ani[num])],
        [lf*np.sin(psi_ani[num]),lf*np.sin(psi_ani[num])+(0.5+40)*np.sin(psi_ani[num]+delta_ani[num])])
    
    car_1_back_wheel.set_data([-(lr+0.5)*np.cos(psi_ani[num]),-(lr-0.5)*np.cos(psi_ani[num])],
        [-(lr+0.5)*np.sin(psi_ani[num]),-(lr-0.5)*np.sin(psi_ani[num])])

    yaw_angle_text.set_text(str(round(psi_ani[num],2))+' rad')
    steer_angle.set_text(str(round(delta_ani[num],2))+' rad')
    body_x_velocity.set_text(str(round(x_dot_ani[num],2))+' m/s')

    return car_1,car_determined,x_dot,yaw_angle,X_position,Y_position,\
            car_1_body,car_1_body_extension,car_1_front_wheel,car_1_front_wheel_extension,\
            car_1_back_wheel,yaw_angle_text,steer_angle,body_x_velocity


frame_amount=len(X_ani)
lf=constants['lf']
lr=constants['lr']

#FIGURE PROPERTIES
fig_x=16
fig_y=9
fig=plt.figure(figsize=(fig_x,fig_y),dpi=120,facecolor=(0.8,0.8,0.8))
n=12
m=12
gs=gridspec.GridSpec(n,m)

# Main trajectory
plt.subplots_adjust(left=0.05,bottom=0.08,right=0.95,top=0.95,wspace=0.15,hspace=0)

ax0=fig.add_subplot(gs[:,0:9],facecolor=(0.9,0.9,0.9))
ax0.grid(True)
plt.xlabel('X-position [m]',fontsize=10)
plt.ylabel('Y-position [m]',fontsize=10)

#plotting ref_trajectory
ref_trajectory=ax0.plot(X_ref,Y_ref,'b',linewidth=1)

#bicycle diagram
car_1,=ax0.plot([],[],'k',linewidth=3)
car_determined,=ax0.plot([],[],'-r',linewidth=1)

# Zoomed vehicle
if trajectory==1:
    ax1=fig.add_subplot(gs[0:6,0:5],facecolor=(0.9,0.9,0.9))
elif trajectory==2:
    ax1=fig.add_subplot(gs[3:9,2:7],facecolor=(0.9,0.9,0.9))
else:
    ax1=fig.add_subplot(gs[2:6,2:5],facecolor=(0.9,0.9,0.9))
ax1.axes.get_xaxis().set_visible(False)
ax1.axes.get_yaxis().set_visible(False)

bbox_props_x_dot=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='b',lw=1.0)
bbox_props_steer_angle=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='r',lw=1.0)
bbox_props_angle=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='k',lw=1.0)

neutral_line=ax1.plot([-50,50],[0,0],'k',linewidth=1)
car_1_body,=ax1.plot([],[],'k',linewidth=3)
car_1_body_extension,=ax1.plot([],[],'--k',linewidth=1)
car_1_back_wheel,=ax1.plot([],[],'r',linewidth=4)
car_1_front_wheel,=ax1.plot([],[],'r',linewidth=4)
car_1_front_wheel_extension,=ax1.plot([],[],'--r',linewidth=1)

plt.xlim(-5,5)
plt.ylim(-4,4)

body_x_velocity=ax1.text(3,-1.5,'',size='10',color='b',bbox=bbox_props_x_dot)
steer_angle=ax1.text(3,-2.5,'',size='10',color='r',bbox=bbox_props_steer_angle)
yaw_angle_text=ax1.text(3,-3.5,'',size='10',color='k',bbox=bbox_props_angle)

body_x_velocity_word=ax1.text(3.7,3.4,'x_dot',size='10',color='b',bbox=bbox_props_x_dot)
steer_angle_word=ax1.text(3.8,2.5,'delta',size='10',color='r',bbox=bbox_props_steer_angle)
yaw_angle_word=ax1.text(4.2,1.6,'Psi',size='10',color='k',bbox=bbox_props_angle)

# x_dot function
ax2=fig.add_subplot(gs[0:3,9:12],facecolor=(0.9,0.9,0.9))
x_dot_reference=ax2.plot(t,x_dot_ref,'-b',linewidth=1)
x_dot,=ax2.plot([],[],'-r',linewidth=1)
ax2.spines['bottom'].set_position(('data',-9999999))
ax2.yaxis.tick_right()
ax2.grid(True)
plt.xlabel('time [s]',fontsize=5)
plt.ylabel('x_dot [m/s]',fontsize=5)
ax2.yaxis.set_label_position("right")

##---------------PSI-FUNCTION-----------------##
ax3=fig.add_subplot(gs[3:6,9:12],facecolor=(0.9,0.9,0.9))
yaw_angle_reference=ax3.plot(t,psi_ref,'-b',linewidth=1)
yaw_angle,=ax3.plot([],[],'-r',linewidth=1)
ax3.spines['bottom'].set_position(('data',-9999999))
ax3.yaxis.tick_right()
ax3.grid(True)
plt.xlabel('time [s]',fontsize=5)
plt.ylabel('Psi [rad]',fontsize=5)
ax3.yaxis.set_label_position("right")

##------------X FUNCTION---------------------##
ax4=fig.add_subplot(gs[6:9,9:12],facecolor=(0.9,0.9,0.9))
X_position_reference=ax4.plot(t,X_ref,'-b',linewidth=1)
X_position,=ax4.plot([],[],'-r',linewidth=1)
ax3.spines['bottom'].set_position(('data',-9999999))
ax3.yaxis.tick_right()
ax3.grid(True)
plt.xlabel('time [s]',fontsize=5)
plt.ylabel('X-Position [m]',fontsize=5)
ax3.yaxis.set_label_position("right")

##---------------Y FUNCTION--------------------##
ax5=fig.add_subplot(gs[9:12,9:12],facecolor=(0.9,0.9,0.9))
Y_position_reference=ax5.plot(t,Y_ref,'-b',linewidth=1)
Y_position,=ax5.plot([],[],'-r',linewidth=1)
ax3.spines['bottom'].set_position(('data',-9999999))
ax3.yaxis.tick_right()
ax3.grid(True)
plt.xlabel('time [s]',fontsize=5)
plt.ylabel('Y-Position [m]',fontsize=5)
ax3.yaxis.set_label_position("right")

car_ani=animation.FuncAnimation(fig,update_plot,frames=frame_amount,interval=20,repeat=True,blit=True)
plt.show()
