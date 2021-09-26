# AeroplaneDynamics
The model for General Dynamics F16 is found in 
[Stevens and Lewis book](https://www.amazon.com/Aircraft-Control-Simulation-Brian-Stevens/dp/0471371459) with code available in Fortran. These dynamics are also available in Scilab, thanks to McFlight page on Github. Now the code is available in Python. Unlike the scilab scripts, the Python files are all available in the same folder.
The following scripts are used to simulate the dynamics of an F16:

Scripts:
* Atmosphere.py: implementation of [International Standard Atmosphere](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf) (ISA) model (ISA) model
* Eqm.py: equations of motion (Force, kinematic, moment and navigation equations)
* Trim.py: script to determine the trim conditions. Which are the conditions at steady flight
* Params.py: Script to determine aerodynamic coefficients of an F16
* engine_f16.py: Script for the engine equation

The python libraries used to simulate the model are:
* Scipy: This is used to minimise the trim_cost_function to determine trim conditions and odeint which is an ordinary differential equation solver used to solve the equations of motion.
* Numpy: Numerical calculations (e.g. cos, sin, arange)
* Pandas: To create empty objects which contain the results
* Matplotlib: Library to create 2D and 3D visualisations

# Simulations
There are two scripts which simulate the the equations of motion:
1. sim_f16.py
2. coordinate_f16.py

## Simulate elevator step
The script sim_f16.py simulates the dynamics of an F16 to a step change in the elevator. The step change in the elevator is plotted along with the acceleration in the z axis (altitude) and the altitude. The results of the simulation is shown in the image below.

<img align="left" width="2000" height="600" src="elevator_step.png">

## Simulate coordinated turn
The other script coordinate_f16.py simulates a coordinated turn of an F16. The x, y and z axis are plotted in a 3 dimensional graph. The results of the simulation is shown in the image below.

<img align="left" width="1000" height="500" src="coordinated_turn_F16.png">

# TODO:
* Linearise the model
* Add a control system
