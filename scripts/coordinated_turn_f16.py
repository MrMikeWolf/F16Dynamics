from params_f16 import load_f16
from numpy import arange, sin, cos
import pandas as pd
from scipy.integrate import odeint
from eqm import eqm
import matplotlib.pyplot as plot
import matplotlib

params = load_f16()
params.xcg = 0.35
params.VT_ftps = 502
params.alt_ft = 0
X0 = [
   502.,
   0.2391101,
   0.0005096,
   1.3665928,
   0.0500909,
   0.,
   0.,
   0.,
   0.,
   0.,
   0.,
   0.,
   64.1323,
   ]
controls= pd.Series()
controls.throttle = 0.835
controls.elev_deg = -1.48
controls.ail_deg = 0.0954
controls.rudder_deg = -0.411
def f16_model(t,X):
    xd, _ = eqm(t, X, controls, params)
    return xd
t = arange(0,180,0.001)
print('Simulating...')
y = odeint(func=f16_model, y0=X0, t=t, tfirst=True)
print('Calculating further outputs...')
nz_g = 0*t
nx_g = 0*t
nzs_g = 0*t
mach = 0*t
thrust_pound = 0*t
for i in range(0,len(t)-1):
    xd,outputs = eqm(t[i], y[:][i], controls, params)
    nz_g[i] = outputs.nz_g
    nx_g[i] = outputs.nx_g
    nzs_g[i] = nx_g[i]*sin(y[i,1])+nz_g[i]*cos(y[i,1])
    mach[i] = outputs.mach
    thrust_pound[i] = outputs.thrust_pound*sin(y[i,4])

x1=y[:,9]
y1=y[:,10]
z1=y[:,11]

font = {'family' : 'Ariel',
        'size'   : 12}

matplotlib.rc('font', **font)    

fig = plot.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x1,y1,z1)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
