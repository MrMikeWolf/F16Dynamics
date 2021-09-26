from params_f16 import load_f16
from trim_f16 import cost_trim_f16
from engine_f16 import tgear
from eqm import eqm
from numpy import sin, cos, zeros
from control import linearize
from scipy.optimize import minimize

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
S = minimize(costf16, S0)['x'] #fminsearch is optimisation using nelder-mead

X0 = [
      params.VT_ftps,    #VT_fps
      S[2],              #alpha_rad
      S[2],              #theta_rad
      0.0,               #q_rps
      params.alt_ft,     #alt_ft
      tgear(S[0]),       #power_perc
     ]

U0 = [
     S[0],
     S[1],
     ]

def sim_f16(X,U):
    controls={}
    controls.throttle = U[0]
    controls.elev_deg = U[1]
    controls.ail_deg = 0.0
    controls.rudder_deg = 0.0
    X_full = zeros(20,0)
    X_full[0] = X[0]
    X_full[1] = X[1]
    X_full[4] = X[2]
    X_full[7] = X[3]
    X_full[11]= X[4]
    X_full[12]= X[5]
    xd_full, outputs = eqm(0, X_full, controls, params)
    xd = xd_full([1, 2, 5, 8, 12, 13])
    y = [
        outputs.nz_g,
        outputs.ny_g,
        outputs.nx_g,
        outputs.Q_lbfpft2,
        outputs.mach,
        outputs.q_rps,
        outputs.alpha_deg,
        outputs.nx_g*sin(X[1]) + outputs.nz_g*cos(X[1])
        ]
    return y, xd

print('Linearizing...')
[A,B,C,D] = linearize(sim_f16, X0, U0)
# ss = syslin("c", A, B, C, D)

# def elev_step_lin(t):
#     if(t<0.5):
#         y = 0.0
#     elif (t>=0.5 & t<=0.53):
#         y = - 1/0.03*(t-0.5)
#     else:
#         y = -1
    
#     return y, xd

# t = range(0,3,0.001)
# print('Simulating linear model...')
# [y,x] = csim(elev_step_lin,t,ss(8,2))
