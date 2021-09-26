from trim_f16 import cost_trim_f16
from params_f16 import load_f16
from engine_f16 import tgear
from eqm import eqm
from scipy.optimize import minimize
import pandas as pd
from scipy.integrate import odeint
from numpy import arange, sin, cos
import matplotlib.pyplot as plot

params = load_f16()
params.xcg = .35
params.coordinated_turn = 0
params.turn_rate_rps = 0.0
params.roll_rate_rps = 0.0
params.pitch_rate_rps = 0.0
params.phi_rad = 0.0
params.gamma_rad = 0.0
params.stability_axis_roll = 0
params.VT_ftps = 502
params.alt_ft = 0
def costf16(x):
    y = cost_trim_f16(x,params)
    return y
S0 = [
     .0,   #throttle 0-1
     0.0,  #elev_deg
     0.0,  #alpha_rad
     #0.0#ail_deg
     #0.0#rudder_deg
     #0.0#beta_rad
     ]
S = minimize(costf16, S0)['x']

X0 = [
      params.VT_ftps,    #VT_fps
      S[2],              #alpha_rad
      0.0,               #beta_rad
      0.0,               #phi_rad
      S[2],              #theta_rad
      0.0,               #psi_rad
      0.0,               #p_rps
      0.0,               #q_rps
      0.0,               #r_rps
      0.0,               #north position ft
      0.0,               #east position ft
      params.alt_ft,     #alt_ft
      tgear(S[0]),       #power_perc
     ]

# PYTHON simulation
controls=pd.Series()

controls.throttle = S[0]
controls.elev_deg = S[1]
controls.ail_deg = 0.0
controls.rudder_deg = 0.0

def f16_model(t,X):
    xd, _ = eqm(t, X, controls, params)
    return xd

def elev_step(t):
    if (t<0.5):
        y = S[1]
    elif (t>=0.5 and t<=0.53):
        y = S[1] - 1/0.03*(t-0.5)
    else:
        y = S[1]-1
    
    return y
t = arange(0,3,0.001)
controls.elev_deg = elev_step
print('Simulating...')
y = odeint(func=f16_model, y0=X0, t=t, tfirst=True)
print('Calculating further outputs...')
nz_g = 0*t
nx_g = 0*t
nzs_g = 0*t
mach = 0*t
thrust_pound = 0*t
for i in range(0,len(t)):
    xd,outputs = eqm(t[i], y[:][i], controls, params)
    nz_g[i] = outputs.nz_g
    nx_g[i] = outputs.nx_g
    nzs_g[i] = nx_g[i]*sin(y[i,1])+nz_g[i]*cos(y[i,1])
    mach[i] = outputs.mach
    thrust_pound[i] = outputs.thrust_pound*sin(y[i,4])

ax1=plot.subplot(311)
ax1.plot(t, [elev_step(ti) for ti in t]);
ax1.set_xlabel('Time(s)');
ax1.set_ylabel('Elevator(deg)')

ax2=plot.subplot(312)
ax2.plot(t, nzs_g);
ax2.set_xlabel('Time(s)')
ax2.set_ylabel('Nz(g)')

ax3=plot.subplot(313)
ax3.plot(t, y[:,11])
ax3.set_xlabel('Time(s)')
ax3.set_ylabel('H(ft)')

