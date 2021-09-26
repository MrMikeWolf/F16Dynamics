from engine_f16 import tgear
from eqm import eqm
from params_f16 import load_f16
from scipy.optimize import minimize
import pandas as pd
from numpy import sqrt, sin, cos, arctan, zeros

def  trim_straight_level(V_ftps, alt_ft=None, xcg=None):
    if not xcg:
        xcg = .35
    
    if not alt_ft:
        alt_ft = 0.0
    
    params = load_f16()
    params.xcg = xcg
    params.coordinated_turn = 0
    params.turn_rate_rps = 0.0
    params.roll_rate_rps = 0.0
    params.pitch_rate_rps = 0.0
    params.phi_rad = 0.0
    params.gamma_rad = 0.0
    params.stability_axis_roll = 0
    params.VT_ftps = V_ftps
    params.alt_ft = alt_ft
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
         
    S = minimize(costf16, S0,'neldder-mead') # fminsearch is a nelder-mead optimisation technique
    
    X = [
          params.VT_ftps,    #VT_fps
          S[2],              #alpha_rad
          0.0,              #beta_rad
          0.0,              #phi_rad
          S[2],              #theta_rad
          0.0,               #psi_rad
          0.0,               #p_rps
          0.0,               #q_rps
          0.0,               #r_rps
          0.0,               #north position ft
          0.0,               #east position ft
          params.alt_ft,     #alt_ft
          tgear(S[0])       #power_perc
         ]
    controls=pd.Series()
    controls.throttle = S[0]
    controls.elev_deg = S[1]
    controls.ail_deg = 0.0
    controls.rudder_deg = 0.0

    return X, controls, params

def  trim_coordinated_turn(V_ftps, alt_ft, turn_rate_rps, gamma_rad=None, xcg=None):
    
    if not xcg:
        xcg = .35
    
    if not gamma_rad:
        gamma_rad = 0.0
    
    params = load_f16()
    params.xcg = xcg
    params.coordinated_turn = 1
    params.turn_rate_rps = turn_rate_rps
    params.roll_rate_rps = 0.0
    params.pitch_rate_rps = 0.0
    params.phi_rad = 0.0
    params.gamma_rad = gamma_rad
    params.stability_axis_roll = 0
    params.VT_ftps = V_ftps
    params.alt_ft = alt_ft
    def costf16(x):
        y = cost_trim_f16(x,params)
        return y
    S0 = [
         .1,   #(1)throttle 0-1
         0.0,  #(2)elev_deg
         0.0,  #(3)alpha_rad
         0.0,  #(4)ail_deg
         0.0,  #(5)rudder_deg
         0.0,  #(6)beta_rad
         ]
    
    S = minimize(costf16, S0,'nelder-mead')
    X = [
          params.VT_ftps,    #VT_fps
          S[2],              #alpha_rad
          S[5],              #beta_rad
          0.0,               #phi_rad
          0.0,               #theta_rad
          0.0,               #psi_rad
          0.0,               #p_rps
          0.0,               #q_rps
          0.0,               #r_rps
          0.0,               #north position ft
          0.0,               #east position ft
          params.alt_ft,     #alt_ft
          tgear(S[0]),       #power_perc
         ]
    X = trim_constraint_f16(X, params)
    controls={}
    controls.throttle = S[0]
    controls.elev_deg = S[1]
    controls.ail_deg = S[3]
    controls.rudder_deg = S[4]
    controls=pd.Series(controls)
    return X, controls, params

def cost_trim_f16(S, params):
    X = zeros((13,1))
    controls=pd.Series()
    controls.throttle = S[0]
    controls.elev_deg = S[1]
    X[1] = S[2]
    if(len(S)>3):
        controls.ail_deg = S[3]
        controls.rudder_deg = S[4]
        X[2] = S[5]
    else:
        controls.ail_deg = 0.0
        controls.rudder_deg = 0.0
        X[2] = 0.0
    
    X[12] = tgear(controls.throttle)
    X = trim_constraint_f16(X, params)
    XD, _ = eqm(0, X, controls, params)
    y = XD[0]**2 + 100*(XD[1]**2 + XD[2]**2) + 10*(XD[6]**2 + XD[7]**2 + XD[8]**2)
    return y

def trim_constraint_f16(X, params):
    X[0] = params.VT_ftps
    X[11] = params.alt_ft
    cos_alpha = cos(X[1])
    sin_alpha = sin(X[1])
    tan_alpha = sin_alpha/cos_alpha
    cos_beta = cos(X[2])
    sin_beta = sin(X[2])
    if (params.coordinated_turn):
        sin_gamma = sin(params.gamma_rad)
        centripetal_acc = params.turn_rate_rps*params.VT_ftps/params.g0_ftps2
        a = 1 - centripetal_acc*tan_alpha*sin_beta
        b = sin_gamma/cos_beta
        c = 1 + (centripetal_acc**2)*cos_beta**2
        #phi_rad, Stevens&Lewis 2nd edition, eq 3.6-5
        X[3] = arctan(centripetal_acc*(cos_beta/cos_alpha)*((a-b**2) + b*tan_alpha*sqrt(c*(1-b**2) + (centripetal_acc**2)*sin_beta**2))/(a**2 - (b**2)*(1 + c*tan_alpha**2)))
        #theta_rad, Stevens&Lewis 2nd edition eq 3.6-3
        a = cos_alpha*cos_beta
        b = sin(X[3])*sin_beta + cos(X[3])*sin_alpha*cos_beta 
        X[4] = arctan((a*b + sin_gamma*sqrt(a**2 - sin_gamma**2 + b**2))/(a**2 - sin_gamma**2))
    elif (params.turn_rate_rps != 0):
        #skidding turn logic
        pass
    else: #non-turning flight
        X[3] = params.phi_rad
        alpha_rad = X[1]
        if (params.phi_rad != 0.0): 
            alpha_rad = -X[1]
        
        if (params.gamma_rad != 0.0):
            sin_gamma_over_cos_beta = sin(params.gamma_rad)/cos_beta
            X[4] = alpha_rad + arctan(sin_gamma_over_cos_beta/sqrt(1.0-sin_gamma_over_cos_beta*sin_gamma_over_cos_beta))
        else:
            X[4] = alpha_rad
        
        X[6] = params.roll_rate_rps
        X[7] = params.pitch_rate_rps
        if (params.stability_axis_roll):
            X[8] = params.roll_rate_rps*sin_alpha/cos_alpha # stability-axis roll
        else:
            X[8] = 0.0                               # body-axis roll
        
    
    X_new = X
    return X_new
