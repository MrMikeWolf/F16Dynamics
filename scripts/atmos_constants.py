# class atmos_constants:
#   def __init__(self):
#     """ 
#     International Standard selfphere, Mean Sea Level Conditions
#     """
#     self.p0_Pa      = 101325.0; # pressure
#     self.rho0_kgpm3 = 1.225; # density
#     self.T0_K       = 288.15; # temperature
#     self.a0_mps     = 340.294; # speed of sound
#     self.g0_mps2    = 9.80665; # acceleration of gravity
#     self.R          = 287.04; # real gas constant m2/K.s2
#     self.r0_m       = 6.356766e9; # Earth radius

# atmos=atmos_constants()
import pandas as pd

atmos=pd.Series({ 'p0_Pa'      : 101325.0, # pressure
        'rho0_kgpm3' : 1.225, # density
        'T0_K'       : 288.15, # temperature
        'a0_mps'     : 340.294, # speed of sound
        'g0_mps2'    : 9.80665, # acceleration of gravity
        'R'          : 287.04, # real gas constant m2/K.s2
        'r0_m'       : 6.356766e9}) # Earth radius