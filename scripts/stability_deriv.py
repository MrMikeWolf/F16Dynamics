from aerodata_f16 import *
from numpy import zeros, pi, sin, cos, array

def stability_deriv(eqm_fun, X0, controls, params, dX=None):
    
    if not dX:
        onedeg_rad = 1/180*pi
        dX = [
              1, #(1)V_ftps
              onedeg_rad, #(2)alpha_rad
              onedeg_rad, #(3)beta_rad
              onedeg_rad, #(4)phi_rad
              onedeg_rad, #(5)theta_rad
              onedeg_rad, #(6)psi_rad
              onedeg_rad, #(7)p_rps
              onedeg_rad, #(8)q_rps
              onedeg_rad, #(9)r_rps
              1, #(10)north position ft
              1, #(11)east position ft
              1, #(12)alt_ft
              1 #(13)power_perc
             ]

    rad2deg = 180/pi
    long={}

    # Alpha derivatives
    CZ_up = CZ((X0[2]+dX[2]/2)*rad2deg, X0[3]*rad2deg, controls.elev_deg)
    CZ_down = CZ((X0[2]-dX[2]/2)*rad2deg, X0[3]*rad2deg, controls.elev_deg)
    long.CZalpha = (CZ_up - CZ_down)/dX[2]
    
    CX_up = CX((X0[2]+dX[2]/2)*rad2deg, controls.elev_deg)
    CX_down = CX((X0[2]-dX[2]/2)*rad2deg, controls.elev_deg)
    long.CXalpha = (CX_up - CX_down)/dX[2]
    
    CM_up = CM((X0[2]+dX[2]/2)*rad2deg, controls.elev_deg)
    CM_down = CM((X0[2]-dX[2]/2)*rad2deg, controls.elev_deg)
    long.CMalpha = (CM_up - CM_down)/dX[2]
    
    X = X0
    X[2] = X0[2] + dX[2]/2
    [XD_up, out_up] = eqm_fun(0, X, controls, params)
    X[2] = X0[2] - dX[2]/2
    [XD_down, out_down] = eqm_fun(0, X, controls, params)
    long.CZalphadot = (out_up.aero_forces(3) - out_down.aero_forces(3))/(XD_up[2] - XD_down[2])
    long.CMalphadot = (out_up.aero_moments(3) - out_down.aero_moments(3))/(XD_up[2] - XD_down[2])

    # Speed derivatives
    X = X0
    X[1] = X0[1]-dX[1]/2
    [XD_down, out_down] = eqm_fun(0, X, controls, params)
    X[1] = X0[1]+dX[1]/2
    [XD_up, out_up] = eqm_fun(0, X, controls, params)
    long.CZv = (out_up.aero_forces(3) - out_down.aero_forces(3))/dX[1]
    long.CXv = (out_up.aero_forces(1) - out_down.aero_forces(1))/dX[1]
    long.CMv = (out_up.aero_moments(2) - out_down.aero_forces(2))/dX[1]
    
    long.Thrust_v = (out_up.thrust_pound - out_down.thrust_pound)/dX[1]
    
    # Pitch derivatives
    D = aerodynamic_damp(X0[2]*rad2deg)
    long.CXq = D[1]
    long.CZq = D[4]
    long.CMq = D[7]
    
    # Elevator
    X = X0
    dElev = 1
    elev_deg_down = controls.elev_deg - dElev/2
    elev_deg_up = controls.elev_deg + dElev/2
    
    CZ_up = CZ(X0[2]*rad2deg, X0[3]*rad2deg, elev_deg_up)
    CZ_down = CZ(X0[2]*rad2deg, X0[3]*rad2deg, elev_deg_down)
    long.CZelev = (CZ_up - CZ_down)/dElev
    
    CX_up = CX(X0[2]*rad2deg, elev_deg_up)
    CX_down = CX(X0[2]*rad2deg, elev_deg_down)
    long.CXelev = (CX_up - CX_down)/dElev
    
    CM_up = CM(X0[2]*rad2deg, elev_deg_up)
    CM_down = CM(X0[2]*rad2deg, elev_deg_down)
    long.CMelev = (CM_up - CM_down)/dElev
    
    # Throttle
    X = X0
    dThrottle = 1
    X[13] = X0[13] - dThrottle/2
    [XD_down, out_down] = eqm_fun(0, X, controls, params)
    X[13] = X0[13] + dThrottle/2
    [XD_up, out_up] = eqm_fun(0, X, controls, params)
    long.Thrust_throttle = (out_up.thrust_pound - out_down.thrust_pound)/dThrottle
    long.CMthrottle = (out_up.aero_moments(2) - out_down.aero_forces(2))/dThrottle
    Q_lbfpft2 = out_down.Q_lbfpft2
    
    # Dimensional derivatives
    qS = Q_lbfpft2*params.geom.wing_ft2
    qSc = qS*params.geom.chord_ft
    m = params.mass.mass_slug
    Iyy = params.mass.AYY

    long.Zalpha = qS/m * long.CZalpha
    long.Xalpha = qS/m * long.CXalpha
    long.Malpha = qSc/Iyy * long.CMalpha
    long.Zalphadot = qSc/(2*m*params.VT_ftps) * long.CZalphadot
    long.Malphadot = qSc/(2*Iyy*params.VT_ftps) * long.CMalphadot
    long.Zelev = qS/m * long.CZelev
    long.Xelev = qS/m * long.CXelev
    long.Melev = qSc/Iyy * long.CMelev
    long.Zq = qS/(2*m*params.VT_ftps) * long.CZq
    long.Mq = qSc/(2*Iyy*params.VT_ftps) * long.CMq
    long.Zv = qS/m * params.VT_ftps * long.CZv
    long.Xv = qS/m * params.VT_ftps * long.CXv
    long.Mv = qSc/Iyy * params.VT_ftps * long.CMv
    long.XTv = long.Thrust_v/m
    long.Xthrottle = long.Thrust_throttle/m
    long.Mthrottle = qS/m * long.CMthrottle
    
    #Matrices for state [alpha q V theta]
    long.E = [[params.VT_ftps-long.Zalphadot, 0, 0, 0],
              [-long.Malphadot, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]]

    long.A = [[long.Zalpha, params.VT_ftps+long.Zq, long.Zv-long.XTv*sin(X0[2]) -params.g0_ftps2*sin(X0[5]-X0[2])],
              [long.Malpha, long.Mq, long.Mv, 0],
              [long.Xalpha, 0, long.Xv+long.XTv*cos(X0[2]) - params.g0_ftps2*cos(X0[5]-X0[2])],
              [0, 1, 0, 0]]

    long.B = [[long.Zelev, -long.Xthrottle*sin(X0[2])],
              [long.Melev, long.Mthrottle],
              [long.Xelev, long.Xthrottle*cos(X0[2])],
              [0, 0]]
    # Turning [alpha q V theta] -> [V alpha theta q]
    T = [[0, 0, 1, 0],
         [1, 0, 0, 0],
         [0, 0, 0, 1],
         [0, 1, 0, 0]]

    long.E = array(T)*array(long.E)*array(T).H #.H is the conjugate transpose
    long.A = array(T)*array(long.A)*array(T).H
    long.B = array(T)*array(long.B)
    long.state_labels = ['V' 'alpha' 'theta' 'q']
    long.input_labels = ['elev' 'throttle']
    
    # TODO: lateral derivatives
    lat = {}
    return long, lat
