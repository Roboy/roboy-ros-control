#%%
from PaBiRoboy_dynamics_derivation import *
#%%
from scipy.integrate import odeint
from sympy.utilities.codegen import codegen
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
x0 = zeros(10)
x0

x0[0] = deg2rad(80)
x0[1] = -deg2rad(80)
x0[2] = -deg2rad(80)
x0[3] = deg2rad(80)
x0[4] = 0

x0

numerical_specified = zeros(6)
#%%
rad2deg(-3.14/2.0)
#%% Jacobian
print ('calculating jacobian')
F0 = ankle_right.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
F0 = Matrix([F0[0], F0[1]])
F0
#%%
L = 1
#constraint = F0 - Matrix([L,0])
J_ankleRight = F0.jacobian([theta0, theta1, theta2, theta3, phi])
J_ankleRight.shape
#%%
F1 = hip_center.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
F1 = Matrix([F1[0], F1[1]])
F1
#%%
J_hipCenter = F1.jacobian([theta0, theta1, theta2, theta3, phi])
J_hipCenter
#%%
J = J_ankleRight.col_join(J_hipCenter)
J
#%%
values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, theta0: x0[0], theta1: x0[1], theta2: x0[2], theta3: x0[3], phi: x0[4]}
Jpinv = J.subs(values).evalf().pinv()
Jpinv
#%%
F2 = F0.col_join(F1)
F2
#np.linalg.pinv(np.array(J.subs(values).evalf()).astype(float))
#%% 
frames_per_sec = 30
final_time = 1

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
#from sympy import lambdify, solve
#M_func = lambdify(coordinates + speeds + constants + specified, mass_matrix) # Create a callable function to evaluate the mass matrix 
#f_func = lambdify(coordinates + speeds + constants + specified, forcing_vector)     # Create a callable function to evaluate the forcing vector       
#%%
kp = np.eye(4)
kp[0,0] = 10
kp[1,1] = 10
kp[2,2] = 10
kp[3,3] = 10
kp
#%%
kpN = np.zeros((4,4),dtype=float)
kpN[1,1] = 10
kpN
x_des = Matrix([L,0,0.7,0.1])
x_des.shape
#%%
i = 0
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
    global i
    r = 0.0                                  # The input force is always zero     
    arguments = np.hstack((x, args))      # States, input, and parameters
    values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, 
              theta0: x[0], theta1: x[1], theta2: x[2], theta3: x[3], phi: x[4],
                omega0: 0, omega1: 0, omega2: 0, omega3: 0, psi: 0}
    Jpinv = J.subs(values).evalf().pinv()
#    N=np.eye(4)-Jpinv*J.subs(values).evalf()
    x_current = F0.subs(values).evalf().col_join(F1.subs(values).evalf())
#    x_des[0] = x_current[0]
#    x_des[2] = x_current[2]
    dq = np.array(Jpinv*(kp*( x_des - x_current ))).astype(float).T[0]# + N*(kpN*(Matrix([0,-deg2rad(80),0,0,0])-Matrix([x[0],x[1],x[2],x[3])))).astype(float).T[0]
    dq = np.hstack((dq,np.zeros(5,dtype=float)))
    if i%10==0:
        print(x_current)
    i = i+1
    return dq
#    dq = np.array(np.linalg.solve(M_func(*arguments),  # Solving for the derivatives
#                  f_func(*arguments))).T[0]
#    return dq
    
    
#%%
print('integrating')
args = (np.hstack((numerical_constants,numerical_specified,)),)
y = odeint(right_hand_side, x0, t, args)#,full_output = 1,mxstep=1000000
#y = np.hstack((y,np.zeros(y.shape)))
print(y)
print('done integrating')
#%%
values = {lower_leg_length: 0.4, upper_leg_length: 0.54, hip_length: 0.2, 
              theta0: deg2rad(90), theta1: deg2rad(0), theta2: deg2rad(160), theta3: deg2rad(0)}
F0.subs(values).evalf()        
#%%
#%% Plot
print("Plotting")
plot(t, rad2deg(y[:, :5]))
xlabel('Time [s]')
ylabel('Angle [deg]')
legend(["${}$".format(vlatex(c)) for c in coordinates])
                     
#plot(t, rad2deg(y[:, 5:]))
#xlabel('Time [s]')
#ylabel('Angular Rate [deg/s]')
#legend(["${}$".format(vlatex(s)) for s in speeds])

#%% Visualization
print("Visualize")
# external
from pydy.viz.shapes import Cylinder, Sphere
from pydy.viz.scene import Scene
from pydy.viz.visualization_frame import VisualizationFrame

# A cylinder will be attached to each link and a sphere to each bob for the
# visualization.
lengths = [lower_leg_length, upper_leg_length, hip_length, hip_length, hip_length, upper_leg_length, lower_leg_length]
bodies = [lower_leg_left, upper_leg_left, hip, hip, hip, upper_leg_right, lower_leg_right]

viz_frames = []
colors = ['yellow','green','red','red','red','green','blue']

for i, (body, particle, mass_center) in enumerate(zip(bodies, particles, mass_centers)):
#        body_shape = Cylinder(name='cylinder{}'.format(i),
#                              radius=0.05,
#                              length=lengths[i],
#                              color='red')
#    
#        viz_frames.append(VisualizationFrame('link_frame{}'.format(i), body,
#                                             body_shape))
            
    particle_shape = Sphere(name='sphere{}'.format(i),
                            radius=0.06,
                            color=colors[i])
                            
    
    viz_frames.append(VisualizationFrame('particle_frame{}'.format(i),
                                         body.frame,
                                         particle,
                                         particle_shape))
                                         
    mass_center_shape = Sphere(name='sphere{}'.format(i),
                            radius=0.02,
                            color='black')
                                         
    viz_frames.append(VisualizationFrame('mass_center_frame{}'.format(i),
                                         body.frame,
                                         mass_center,
                                         mass_center_shape))
                                             
target_shape = Sphere(name='sphere{}'.format(i),
                            radius=0.02,
                            color='green')    

target_right_leg = Point('target_right_leg')
target_right_leg.set_pos(origin, (x_des[2] * inertial_frame.x)+(x_des[3] * inertial_frame.y))                              

                                     
viz_frames.append(VisualizationFrame('target_frame_right',
                                     inertial_frame,
                                     target_right_leg,
                                     target_shape))

## Now the visualization frames can be passed in to create a scene.
scene = Scene(inertial_frame, origin, *viz_frames)

# Provide the data to compute the trajectories of the visualization frames.
scene.constants = dict(zip(constants, numerical_constants))
scene.states_symbols = coordinates+speeds
scene.states_trajectories = y

scene.display()
#%% export to c header
t = symbols('t')
a0 = ankle_left.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
k0 = knee_left.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
hl = hip_left.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
hc = hip_center.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
hr = hip_right.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
k1 = knee_right.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
a1 = ankle_right.pos_from(origin).express(inertial_frame).simplify().to_matrix(inertial_frame)
[(c_name, c_code), (h_name, c_header)] = codegen( 
    [("Jacobian",J), 
     ("ankle_right_hip_center",F2), 
     ("ankle_left", a0),
     ("knee_left", k0),
     ("hip_left", hl),
     ("hip_center", hc),
     ("hip_right", hr),
     ("knee_right", k1),
     ("ankle_right",a1)
    ] ,
    "C", "PaBiRoboy_DanceControl", project='PaBiRoboy_DanceControl',global_vars=(t,lower_leg_length, upper_leg_length, hip_length, theta0,theta1,theta2,theta3), 
    header=True, empty=False)
print(c_code)