#%%
from PaBiRoboy_dynamics_derivation import *
#%%
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
                             
                             9.81],  # acceleration due to gravity [m/s^2]
                            ) 
numerical_constants

specified = [ankle_torque0, knee_torque0, hip_torque0, hip_torque1, knee_torque1, ankle_torque1]
#%%
x0 = zeros(8)
x0

x0[0] = deg2rad(0.1)
x0[1] = deg2rad(0.1)
x0[2] = deg2rad(0.1)
x0[3] = deg2rad(0.1)

x0

numerical_specified = zeros(6)
#%% Jacobian
print ('calculating jacobian')
F0 = ankle_left.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
F0 = Matrix([F0[0], F0[1]])
F0
##%%
#F00 = ankle_left.vel(inertial_frame).simplify().to_matrix(inertial_frame)
#F00 = Matrix([F00[0], F00[1]])
#F00
#%%
F1 = ankle_right.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
F1 = Matrix([F1[0], F1[1]])
F1
#%%
J_ankleLeft = F0.jacobian([theta0, theta1, theta2, theta3])
J_ankleLeft
#%%
J_ankleRight = F1.jacobian([theta0, theta1, theta2, theta3])
J_ankleRight
#%%
J = J_ankleLeft.col_join(J_ankleRight)
J
##
values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, theta0: x0[0], theta1: x0[1], theta2: x0[2], theta3: x0[3]}
Jpinv = J.subs(values).evalf().pinv()
Jpinv
#np.linalg.pinv(np.array(J.subs(values).evalf()).astype(float))
#%% 
frames_per_sec = 60
final_time = 5

t = linspace(0.0, final_time, final_time * frames_per_sec)

#%%
#right_hand_side = generate_ode_function(forcing_vector, coordinates, speeds, 
#                                        constants, mass_matrix=mass_matrix, 
#                                        specifieds=specified, generator='cython')
#
#args = {'constants': numerical_constants,
#        'specified': numerical_specified}
#                                        
#right_hand_side(x0, 0.0, numerical_specified, numerical_constants)        
##%%
#y = odeint(right_hand_side, x0, t, args=(numerical_specified, numerical_constants))
#y

#%%
from sympy import lambdify, solve
M_func = lambdify(coordinates + speeds + constants + specified, mass_matrix) # Create a callable function to evaluate the mass matrix 
f_func = lambdify(coordinates + speeds + constants + specified, forcing_vector)     # Create a callable function to evaluate the forcing vector       
#%%
kp = 10
kpN = np.zeros((4,4),dtype=float)
kpN[0,0] = 1
kpN
x_des = Matrix([0.4,-0.3,-0.2,-0.5])
x_des.shape
#%%

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
    values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, 
              theta0: x[0], theta1: x[1], theta2: x[2], theta3: x[3],
                omega0: 0, omega1: 0, omega2: 0, omega3: 0}
#    Jpinv = np.linalg.pinv(np.array(J.subs(values).evalf()).astype(float))
    Jpinv = J.subs(values).evalf().pinv()
#    N=np.eye(4)-Jpinv*J.subs(values).evalf()
    x_current = F0.subs(values).evalf().col_join(F1.subs(values).evalf())
    dq = np.array(Jpinv*(kp*( x_des - x_current ))).astype(float).T[0]# + kp*( x_des - x_current ))+ N*(kpN*(Matrix([0,0,0,0,0])-Matrix([x[0],x[1],x[2],x[3],x[4]])))).astype(float).T[0]
    print(x_current)
    dq = np.hstack((dq,np.zeros(4,dtype=float)))
    return dq
##    print(x)

#    dq = np.array(np.linalg.solve(M_func(*arguments),  # Solving for the derivatives
#                  f_func(*arguments))).T[0]
#    return dq
    
    

#%%
args = (np.hstack((numerical_constants,numerical_specified,)),)
args[0].shape

#%%
print('integrating')
y = odeint(right_hand_side, x0, t, args)#,full_output = 1,mxstep=1000000
#y = np.hstack((y,np.zeros(y.shape)))
print(y)
print('done integrating')
#%%
values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, 
              theta0: deg2rad(90), theta1: deg2rad(0), theta2: deg2rad(160), theta3: deg2rad(0)}
F0.subs(values).evalf()        
##%%
#
#Jpinv = J.subs(values).pinv()
#Jpinv
###    N=np.eye(4)-Jpinv*J;
#x_current = F0.subs(values).evalf()
#x_current
#dq = np.array(Jpinv*(x_current + kp*(Matrix([-0.0632292870069145, -0.925719287831476]) - x_current ))).astype(float)
#dq
#%%
#Jpinv = np.linalg.pinv(np.array(J.subs(values).evalf()).astype(float))
#N=np.eye(5)-Jpinv*J
#N.shape
#x_current = F0.subs(values).evalf()
#values = {hip_offset_x: 0, hip_offset_y: 1.0, lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, theta0: 0.0, theta1: 0.1, theta2: 0.1, theta3: 0.1, theta4: 0.1}
#Jpinv = np.linalg.pinv(np.array(J.subs(values).evalf()).astype(float))
#np.array(Jpinv*(x_current + kp*(Matrix([-0.0632292870069145, -0.93800166611121]) - x_current )) + N*(kpN*(Matrix([0,0,0,0,0])-Matrix([0,0,0,0,0])))).astype(float).T[0]