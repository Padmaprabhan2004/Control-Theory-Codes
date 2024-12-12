import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import matplotlib.gridspec as gridspec

radius=5
bottom=0
final_volume=100
dvol=10
width_ratio=1
dt=0.04
t0=0
t_end=50
frame_amount=int(t_end/dt)
t=np.arange(t0,t_end+dt,dt)
density=1000

kp1=1000
kp2=2000
kp3=3000

vol_init_1=30
vol_ref_1=70
vol_init_2=30
vol_ref_2=70
vol_init_3=30
vol_ref_3=70

#tank1
volr1=np.zeros(len(t))
volume_tank1=np.zeros(len(t))
volume_tank1[0]=vol_init_1
error1=np.zeros(len(t))
m_dot1=kp1*error1


#tank2
volr2=np.zeros(len(t))
volume_tank2=np.zeros(len(t))
volume_tank2[0]=vol_init_2
error2=np.zeros(len(t))
m_dot2=kp2*error2

#tank3
volr3=vol_init_3+2*np.sin(np.pi*(0.05*t)*t)
volume_tank3=np.zeros(len(t))
error3=np.zeros(len(t))
m_dot3=kp3*error3

for i in range(1,len(t)):
    #defining the movement of each reference except 3, which is already decided
    if i<300:
        volr1[i]=vol_init_1
        volr2[i]=vol_init_2+3*t[i]
    elif i<600:
        volr1[i]=20
        volr2[i]=volr2[i-1]+2*np.sin(2*np.pi*t[i])
    elif i<900:
        volr1[i]=90
        volr2[i]=vol_init_2+0.05*(t[i]**2)
    else:
        volr1[i]=50
        volr2[i]=volr2[i-1]-2*dt
    

    error1[i-1]=volr1[i-1]-volume_tank1[i-1]
    error2[i-1]=volr2[i-1]-volume_tank2[i-1]
    error3[i-1]=volr3[i-1]-volume_tank3[i-1]

    m_dot1[i]=kp1*error1[i-1]
    m_dot2[i]=kp2*error2[i-1]
    m_dot3[i]=kp3*error3[i-1]

    volume_tank1[i]=volume_tank1[i-1]+(1/density)*dt*0.5*(m_dot1[i]+m_dot1[i-1])
    volume_tank2[i]=volume_tank2[i-1]+(1/density)*dt*0.5*(m_dot2[i]+m_dot2[i-1])
    volume_tank3[i]=volume_tank3[i-1]+(1/density)*dt*0.5*(m_dot3[i]+m_dot3[i-1])


def update(num):
    if num>=len(volume_tank1):
        num=len(volume_tank1)-1
    
    vtank1.set_data([0, 0], [-63, volume_tank1[num]-76])
    vref1.set_data([-radius * width_ratio, radius * width_ratio], [volr1[num], volr1[num]])
    volt1_line.set_data(t[0:num], volume_tank1[0:num])
    volr1_line.set_data([t0, t_end], [volr1[num], volr1[num]])

    vtank2.set_data([0, 0], [-63, volume_tank2[num] - 76])
    vref2.set_data([-radius * width_ratio, radius * width_ratio], [volr2[num], volr2[num]])
    volt2_line.set_data(t[0:num], volume_tank2[0:num])
    volr2_line.set_data([t0, t_end], [volr2[num], volr2[num]])

    vtank3.set_data([0, 0], [-63, volume_tank3[num] - 76])
    vref3.set_data([-radius * width_ratio, radius * width_ratio], [volr3[num], volr3[num]])
    volt3_line.set_data(t[0:num], volume_tank3[0:num])
    volr3_line.set_data([t0, t_end], [volr3[num], volr3[num]])

    return (
        vtank1, vref1, volt1_line, volr1_line,
        vtank2, vref2, volt2_line, volr2_line,
        vtank3, vref3, volt3_line, volr3_line
    )

#main figure
fig=plt.figure(figsize=(16,9),dpi=120,facecolor=(0.8,0.8,0.8))
gs=gridspec.GridSpec(2,3)

#object for tank1
ax0=fig.add_subplot(gs[0,0],facecolor=(0.9,0.9,0.9))
vref1,=ax0.plot([],[],'r',linewidth=2)
vtank1,=ax0.plot([],[],'royalblue',linewidth=260,zorder=0)
plt.xlim(-radius*width_ratio,radius*width_ratio)
plt.ylim(bottom,final_volume)
plt.xticks(np.arange(-radius,radius+1,radius))
plt.yticks(np.arange(bottom,final_volume+dvol,dvol))
plt.ylabel('Tank Volume')
plt.title('TANK 1')

#object for tank2
ax1=fig.add_subplot(gs[0,1],facecolor=(0.9,0.9,0.9))
vref2,=ax1.plot([],[],'r',linewidth=2)
vtank2,=ax1.plot([],[],'royalblue',linewidth=260,zorder=0)
plt.xlim(-radius*width_ratio,radius*width_ratio)
plt.ylim(bottom,final_volume)
plt.xticks(np.arange(-radius,radius+1,radius))
plt.yticks(np.arange(bottom,final_volume+dvol,dvol))
plt.ylabel('Tank Volume')
plt.title('TANK 2')

#object for tank3
ax2=fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
vref3,=ax2.plot([],[],'r',linewidth=2)
vtank3,=ax2.plot([],[],'royalblue',linewidth=260,zorder=0)
plt.xlim(-radius*width_ratio,radius*width_ratio)
plt.ylim(bottom,final_volume)
plt.xticks(np.arange(-radius,radius+1,radius))
plt.yticks(np.arange(bottom,final_volume+dvol,dvol))
plt.ylabel('Tank Volume')
plt.title('TANK 3')

#volume function graph for all three tanks
ax3=fig.add_subplot(gs[1,:],facecolor=(0.9,0.9,0.9))
volr1_line,=ax3.plot([],[],'r',linewidth=2)
volr2_line,=ax3.plot([],[],'r',linewidth=2)
volr3_line,=ax3.plot([],[],'r',linewidth=2)

volt1_line,=ax3.plot([],[],'r',linewidth=4,label='Tank1')
volt2_line,=ax3.plot([],[],'g',linewidth=4,label='Tank2')
volt3_line,=ax3.plot([],[],'b',linewidth=4,label='Tank3')
plt.xlim(0,t_end)
plt.ylim(bottom,final_volume)
plt.ylabel('Tank Volume')
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')

plane_ani=animation.FuncAnimation(fig,update,frames=frame_amount,interval=20,repeat=True,blit=True)
plt.show()