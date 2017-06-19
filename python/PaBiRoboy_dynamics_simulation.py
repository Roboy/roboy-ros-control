#%%
from PaBiRoboy_dynamics_derivation import *
from scipy.integrate import odeint
from pydy.codegen.ode_function_generators import generate_ode_function
from matplotlib.pyplot import plot, legend, xlabel, ylabel, rcParams
rcParams['figure.figsize'] = (14.0, 6.0)
import rospy

#%% Simulation
print("Simulating")
constants = [lower_leg_length,
             upper_leg_length,
             hip_length,
             
             lower_leg_mass,
             upper_leg_mass,
             hip_mass,
             
             lower_leg_inertia,
             upper_leg_inertia,
             hip_inertia,
             
             hip_offset_x,
             hip_offset_y,
             
             g]
constants

numerical_constants = array([0.42,  # lower_leg_length [m]
                             0.54, # upper_leg_length [m]
                             0.2, # hip_length
                                                          
                             1.0,  # lower_leg_mass [kg]
                             1.5,  # upper_leg_mass [kg]
                             2.0,  # hip_mass [kg]
                             
                             0.1,  # lower_leg_inertia [kg*m^2]                             
                             0.2,  # upper_leg_inertia [kg*m^2]
                             0.1,  # hip_inertia [kg*m^2]
                             
                             0.1,
                             0.6,
                             
                             9.81],  # acceleration due to gravity [m/s^2]
                            ) 
numerical_constants

coordinates = [theta0, theta1, theta2, theta3, theta4]
coordinates

speeds = [omega0, omega1, omega2, omega3, omega4]
speeds

specified = [ankle_torque0, knee_torque0, hip_torque0, hip_torque1, knee_torque1, ankle_torque1]
#%%
x0 = zeros(10)
x0

x0[0] = 0
x0[1] = deg2rad(0)
x0[2] = deg2rad(45)
x0[3] = deg2rad(0)
x0[4] = deg2rad(45)
x0

numerical_specified = zeros(6)
#%% Jacobian
print ('calculating jacobian')
F0 = ankle_left.pos_from(hip_center).express(inertial_frame).simplify().to_matrix(inertial_frame)
F0 = Matrix([F0[0], F0[1]])
F0

J_ankleLeft = F0.jacobian([theta1, theta2, theta3, theta4])
J_ankleLeft

J = J_ankleLeft
J

values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, theta0: 0.0, theta1: 0.1, theta2: 0.1, theta3: 0.1, theta4: 0.1}
J.subs(values).pinv()
##%% 
#right_hand_side = generate_ode_function(forcing_vector, coordinates, speeds, 
#                                        constants, mass_matrix=mass_matrix, 
#                                        specifieds=specified, generator='cython')
#
#args = {'constants': numerical_constants,
#        'specified': numerical_specified}
#frames_per_sec = 60
#final_time = 5.0
#
#t = linspace(0.0, final_time, final_time * frames_per_sec)
#                                        
#right_hand_side(x0, 0.0, numerical_specified, numerical_constants)        
##%%
#y = odeint(right_hand_side, x0, t, args=(numerical_specified, numerical_constants))
#y.shape
#y    
#%%
from sympy import lambdify, solve
M_func = lambdify(coordinates + speeds + constants + specified, mass_matrix) # Create a callable function to evaluate the mass matrix 
f_func = lambdify(coordinates + speeds + constants + specified, forcing_vector)     # Create a callable function to evaluate the forcing vector       
def right_hand_side(x, t, args):
    """Returns the derivatives of the states.

    Parameters
    ----------
    x : ndarray, shape(2 * (n + 1))
        The current state vector.
    t : float
        The current time.
    args : ndarray
        The constants.

    Returns
    -------
    dx : ndarray, shape(2 * (n + 1))
        The derivative of the state.
    
    """
    r = 0.0                                  # The input force is always zero     
    arguments = np.hstack((x, args))      # States, input, and parameters
    dq = np.array(np.linalg.solve(M_func(*arguments),  # Solving for the derivatives
                  f_func(*arguments))).T[0]
    
    return dq

#%%
args = (np.hstack((numerical_constants,numerical_specified)),)
args
#%%
y = odeint(right_hand_side, x0, t, args)
y.shape
y
#%%
help(right_hand_side)